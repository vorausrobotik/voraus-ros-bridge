use log::info;
use opcua::server::callbacks;
use opcua::server::prelude::{
    Config, DateTime, EventNotifier, MethodBuilder, NodeId, ObjectBuilder, Server, ServerConfig,
    Variable,
};
use opcua::server::session::SessionManager;
use opcua::sync::RwLock;
use opcua::types::{CallMethodRequest, CallMethodResult, ObjectId, StatusCode};
use std::path::PathBuf;
use std::sync::Arc;
use std::thread;

pub async fn run_rapid_clock() -> thread::JoinHandle<()> {
    thread::spawn(|| {
        rapid_clock();
    })
}

fn rapid_clock() {
    opcua::console_logging::init();

    let mut server =
        Server::new(ServerConfig::load(&PathBuf::from("tests/resources/clock.conf")).unwrap());

    let questionable_namespace = 1u16;

    add_timed_variable(&mut server, questionable_namespace);
    add_no_op_method(&mut server, questionable_namespace);

    server.run();
}

fn add_timed_variable(server: &mut Server, namespace: u16) {
    // These will be the node ids of the new variables
    let ticks_since_launch_node_id = NodeId::new(namespace, 100111);

    let address_space = server.address_space();

    // The address space is guarded so obtain a lock to change it
    {
        let mut address_space = address_space.write();

        let rapid_folder_id = address_space
            .add_folder("Rapid", "Rapid", &NodeId::objects_folder_id())
            .unwrap();

        let _ = address_space.add_variables(
            vec![Variable::new(
                &ticks_since_launch_node_id,
                "joint_positions",
                "joint_positions",
                vec![0.0f64; 6],
            )],
            &rapid_folder_id,
        );
    }

    {
        server.add_polling_action(10, move || {
            let mut address_space = address_space.write();
            let now = DateTime::now();
            let ticks_in_100_ns = now.ticks();
            let _ = address_space.set_variable_value(
                ticks_since_launch_node_id.clone(),
                vec![ticks_in_100_ns as f64; 6],
                &now,
                &now,
            );
        });
    }
}

fn add_no_op_method(server: &mut Server, namespace: u16) {
    let address_space = server.address_space();
    let mut address_space = address_space.write();

    let object_id = NodeId::new(namespace, 100182);
    ObjectBuilder::new(&object_id, "Functions", "Functions")
        .event_notifier(EventNotifier::SUBSCRIBE_TO_EVENTS)
        .organized_by(ObjectId::ObjectsFolder)
        .insert(&mut address_space);

    // NoOp has 0 inputs and 0 outputs
    let method_id = NodeId::new(namespace, 100263);
    MethodBuilder::new(&method_id, "NoOp", "NoOp")
        .component_of(object_id.clone())
        .callback(Box::new(NoOp))
        .insert(&mut address_space);
}

struct NoOp;

impl callbacks::Method for NoOp {
    fn call(
        &mut self,
        _session_id: &NodeId,
        _session_map: Arc<RwLock<SessionManager>>,
        _request: &CallMethodRequest,
    ) -> Result<CallMethodResult, StatusCode> {
        info!("NoOp method called");
        Ok(CallMethodResult {
            status_code: StatusCode::Good,
            input_argument_results: None,
            input_argument_diagnostic_infos: None,
            output_arguments: None,
        })
    }
}
