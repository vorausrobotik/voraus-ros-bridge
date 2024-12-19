use std::sync::{Arc, Mutex};

use log::info;
use opcua::types::{NodeId, Variant};
use std_srvs::srv::{Empty_Request, Empty_Response};
use voraus_interfaces::srv::{MoveJoints_Request, MoveJoints_Response};

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
        info!("ROS service call: Enable impedance control");
        let object_id = NodeId::new(1, 100182);
        let method_id = NodeId::new(1, 100263);
        self.opc_ua_client
            .lock()
            .unwrap()
            .call_method(object_id, method_id, None::<Vec<Variant>>);
        Empty_Response {
            structure_needs_at_least_one_member: 0,
        }
    }

    pub fn disable_impedance_control(
        &self,
        _request_header: &rclrs::rmw_request_id_t,
        _request: Empty_Request,
    ) -> Empty_Response {
        info!("ROS service call: Disable impedance control");
        let object_id = NodeId::new(1, 100182);
        let method_id = NodeId::new(1, 100264);
        self.opc_ua_client
            .lock()
            .unwrap()
            .call_method(object_id, method_id, None::<Vec<Variant>>);
        Empty_Response {
            structure_needs_at_least_one_member: 0,
        }
    }

    pub fn move_joints(
        &self,
        _request_header: &rclrs::rmw_request_id_t,
        request: MoveJoints_Request,
    ) -> MoveJoints_Response {
        info!("ROS service call: Move Joints");
        let object_id = NodeId::new(1, 100210);
        let method_id = NodeId::new(1, 100211);
        self.opc_ua_client.lock().unwrap().call_method(
            object_id,
            method_id,
            Some(vec![
                request.relative.into(),
                request.target_reference_cs.into(),
                request.target_coordinate.into(),
                request.arriving_cs.into(),
                request.velocity_scaling.into(),
                request.with_blending.into(),
                request.blending_parameter.into(),
                request.config_vector.into(),
                request.command_id.into(),
                request.manual_mode.into(),
            ]),
        );
        MoveJoints_Response {
            structure_needs_at_least_one_member: 0,
        }
    }
}
