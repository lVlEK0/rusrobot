extern crate gcc;

fn main(){
    gcc::Build::new()
        .file("src/c/ev3api_brick.c")
        .file("src/c/ev3api_motor.c")
        .file("src/c/ev3api_sensor.c")
        .file("src/c/vasyslog.c")
        .include("src");
}
