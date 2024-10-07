use std::sync::{Arc, Mutex};

use log::info;
use opcua::types::NodeId;
use std_srvs::srv::{Empty_Request, Empty_Response};

use crate::opc_ua_client::OPCUAClient;

pub struct ROSServices {
    opc_ua_client: Arc<Mutex<OPCUAClient>>,
}

impl ROSServices {
    pub fn new(opc_ua_client: Arc<Mutex<OPCUAClient>>) -> Self {
        Self { opc_ua_client }
    }

    pub fn enable_impedance_control(
        &self,
        _request_header: &rclrs::rmw_request_id_t,
        _request: Empty_Request,
    ) -> Empty_Response {
        info!("ROS service called");
        let object_id = NodeId::new(1, 100182);
        let method_id = NodeId::new(1, 100263);
        self.opc_ua_client
            .lock()
            .unwrap()
            .call_method(object_id, method_id, None::<Vec<()>>);
        Empty_Response {
            structure_needs_at_least_one_member: 0,
        }
    }
}
