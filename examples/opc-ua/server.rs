use opcua::server::prelude::*;

fn main() {
    let server: Server = ServerBuilder::new()
        .application_name("Demo Server")
        .application_uri("urn:server_uri")
        .discovery_urls(vec!["opc.tcp://localhost:4855/".to_string()])
        .create_sample_keypair(true)
        .trust_client_certs()
        .pki_dir("./pki-server")
        .discovery_server_url(None)
        .host_and_port("localhost", 4855)
        .endpoint(
            "none",
            ServerEndpoint::new(
                "/",
                SecurityPolicy::None,
                MessageSecurityMode::None,
                &vec![String::from("ANONYMOUS")],
            ),
        )
        .server()
        .unwrap();

    let address_space = server.address_space();

    let folder_id = address_space.write()
        .add_folder("Variables", "Variables", &NodeId::objects_folder_id())
        .unwrap();

    let node_id = NodeId::new(1, "MyVar");
    VariableBuilder::new(&node_id, "MyVar", "MyVar")
    .organized_by(&folder_id)
    .value(0u8)
    .insert(&mut address_space.write());

    let now = DateTime::now();
    let value = 123.456f64;
    let _ = address_space.write().set_variable_value(node_id, value, &now, &now);

    server.run();

    // println!("0");
    // let ns = {
    //         let address_space = server.address_space();
    //         let mut address_space = address_space.write();
    //         address_space.register_namespace("urn:demo-server").unwrap()
    //     };
    // println!("1");
    // let static_folder_id = {
    //     let address_space = server.address_space();
    //     let mut address_space = address_space.write();
    //     address_space
    //         .add_folder("Static", "Static", &NodeId::objects_folder_id())
    //         .unwrap()
    // };
    // println!("2");
    // let address_space = server.address_space();
    // let mut address_space = address_space.write();

    // // Create a folder under static folder
    // let folder_id = address_space
    //     .add_folder("Scalar", "Scalar", &static_folder_id)
    //     .unwrap();

    // println!("3");
    // let value = 42;
    // VariableBuilder::new(&NodeId::new(ns,"Float".to_string()), "Float", "Float")
    //     .data_type(DataTypeId::Float)
    //     .value(value)
    //     .organized_by(&folder_id)
    //     .insert(&mut server.address_space().write());
    // println!("Starting OPC UA server.");
    // server.run();
} //
