mod simple_opc_ua_subscriber;

fn main() {
    println!("Hello, voraus!");
    simple_opc_ua_subscriber::launch_subscriber().unwrap();
}
