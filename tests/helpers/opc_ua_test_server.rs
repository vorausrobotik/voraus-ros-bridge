use log::info;
use opcua::server::callbacks;
use opcua::server::prelude::{
    Config, DateTime, EventNotifier, MethodBuilder, NodeId, ObjectBuilder, Server, ServerConfig,
    Variable,
};
use opcua::server::session::SessionManager;
use opcua::sync::RwLock;
use opcua::types::{
    Array, CallMethodRequest, CallMethodResult, ObjectId, StatusCode, Variant, VariantTypeId,
};
use std::path::PathBuf;
use std::sync::mpsc::Sender;
use std::sync::Arc;
use std::thread::{self, JoinHandle};

pub struct OPCUATestServer {
    server: Arc<RwLock<Server>>,
    run_handle: Option<JoinHandle<()>>,
}

impl OPCUATestServer {
    pub fn new(assertion_tx: Sender<String>) -> OPCUATestServer {
        let questionable_namespace = 1;
        let mut server = Server::new(
            ServerConfig::load(&PathBuf::from("tests/resources/opc_ua_test_server.conf")).unwrap(),
        );
        add_variables(&mut server, questionable_namespace);
        add_methods(&mut server, questionable_namespace, assertion_tx);
        let server = Arc::new(RwLock::new(server));
        let server_cpy = Arc::clone(&server);

        let run_handle = thread::spawn(move || {
            Server::run_server(Arc::clone(&server_cpy));
        });
        OPCUATestServer {
            server,
            run_handle: Some(run_handle),
        }
    }
}

impl Drop for OPCUATestServer {
    fn drop(&mut self) {
        let server_cpy = self.server.clone();
        server_cpy.try_write().unwrap().abort();
        if let Some(run_handle) = self.run_handle.take() {
            if let Err(err) = run_handle.join() {
                eprintln!("Error while joining the run thread: {:?}", err);
            }
        }
    }
}

fn add_variables(server: &mut Server, namespace: u16) {
    add_variable_stub(
        server,
        NodeId::new(namespace, 100111),
        "position".to_string(),
    );
    add_variable_stub(
        server,
        NodeId::new(namespace, 100115),
        "velocity".to_string(),
    );
    add_variable_stub(server, NodeId::new(namespace, 100113), "effort".to_string());
    add_variable_stub(
        server,
        NodeId::new(namespace, 100707),
        "tcp_pose".to_string(),
    );
    add_variable_stub(
        server,
        NodeId::new(namespace, 100710),
        "tcp_quaternion".to_string(),
    );
    add_variable_stub(
        server,
        NodeId::new(namespace, 100708),
        "tcp_twist".to_string(),
    );
    add_variable_stub(
        server,
        NodeId::new(namespace, 100711),
        "tcp_wrench".to_string(),
    );
}
fn add_methods(server: &mut Server, namespace: u16, assertion_tx: Sender<String>) {
    add_method_stub(
        server,
        NodeId::new(namespace, 100182),
        NodeId::new(namespace, 100263),
        None,
        None,
        "impedance_control/enable".to_string(),
        assertion_tx.clone(),
    );
    add_method_stub(
        server,
        NodeId::new(namespace, 100182),
        NodeId::new(namespace, 100264),
        None,
        None,
        "impedance_control/disable".to_string(),
        assertion_tx.clone(),
    );
    add_method_stub(
        server,
        NodeId::new(namespace, 100210),
        NodeId::new(namespace, 100211),
        Some(vec![
            Variant::Boolean(false),
            Variant::UInt32(0),
            // The opcua crate creates an array with dimension = Some([]) when decoding an empty array.
            // The default trait used in the ros service caller generates an empty vector so this is the workaround
            // to still have a useful assertion.
            Array::new_multi(VariantTypeId::Double, Vec::new(), Vec::new())
                .unwrap()
                .into(),
            Variant::UInt32(0),
            Variant::Double(0.0),
            Variant::Boolean(false),
            Variant::Double(0.0),
            Array::new_multi(VariantTypeId::Int32, Vec::new(), Vec::new())
                .unwrap()
                .into(),
            Variant::UInt32(0),
            Variant::Boolean(false),
        ]),
        None,
        "move_joints".to_string(),
        assertion_tx.clone(),
    );
}

/// Creates an OPC UA variable that is monotonously increasing.
fn add_variable_stub(server: &mut Server, node_id: NodeId, identifier: String) {
    let address_space = server.address_space();

    // The address space is guarded so obtain a lock to change it
    {
        let mut address_space = address_space.write();

        let joint_position_stub_folder_id = address_space
            .add_folder(
                identifier.clone(),
                identifier.clone(),
                &NodeId::objects_folder_id(),
            )
            .unwrap();

        let _ = address_space.add_variables(
            vec![Variable::new(
                &node_id,
                identifier.clone(),
                identifier.clone(),
                vec![0.0f64; 6],
            )],
            &joint_position_stub_folder_id,
        );
    }

    {
        server.add_polling_action(10, move || {
            let mut address_space = address_space.write();
            let now = DateTime::now();
            let ticks_in_100_ns = now.ticks();
            let _ = address_space.set_variable_value(
                node_id.clone(),
                vec![ticks_in_100_ns as f64; 6],
                &now,
                &now,
            );
        });
    }
}

fn add_method_stub(
    server: &mut Server,
    object_id: NodeId,
    method_id: NodeId,
    input_args: Option<Vec<Variant>>,
    output_args: Option<Vec<Variant>>,
    assertion_str: String,
    assertion_tx: Sender<String>,
) {
    let address_space = server.address_space();
    let mut address_space = address_space.write();

    if !address_space.node_exists(&object_id) {
        ObjectBuilder::new(&object_id, "Functions", "Functions")
            .event_notifier(EventNotifier::SUBSCRIBE_TO_EVENTS)
            .organized_by(ObjectId::ObjectsFolder)
            .insert(&mut address_space);
    }

    MethodBuilder::new(&method_id, "NoOp", "NoOp")
        .component_of(object_id.clone())
        .callback(Box::new(NoOp {
            assertion_str,
            input_args,
            output_args,
            assertion_tx,
        }))
        .insert(&mut address_space);
}

struct NoOp {
    assertion_str: String,
    input_args: Option<Vec<Variant>>,
    output_args: Option<Vec<Variant>>,
    assertion_tx: Sender<String>,
}

impl callbacks::Method for NoOp {
    fn call(
        &mut self,
        _session_id: &NodeId,
        _session_map: Arc<RwLock<SessionManager>>,
        request: &CallMethodRequest,
    ) -> Result<CallMethodResult, StatusCode> {
        match request.input_arguments == self.input_args {
            true => {
                info!("{}", self.assertion_str);
                self.assertion_tx.send(self.assertion_str.clone()).unwrap()
            }
            false => {
                panic!(
                    "Unexpected input args. Expected {:?} but got {:?}",
                    self.input_args, request.input_arguments
                );
            }
        }

        Ok(CallMethodResult {
            status_code: StatusCode::Good,
            input_argument_results: None,
            input_argument_diagnostic_infos: None,
            output_arguments: self.output_args.clone(),
        })
    }
}
