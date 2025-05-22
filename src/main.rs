use std::time::{SystemTime, UNIX_EPOCH};

use chrono::{Local, Timelike};
use vello::{kurbo::{Affine, Circle, Ellipse, Line, RoundedRect, Stroke}, peniko::Color, Scene};

mod render;
mod simulation;
fn render(scene: &mut Scene) {
    // Draw an outlined rectangle
    let stroke = Stroke::new(6.0);
    let rect = RoundedRect::new(10.0, 10.0, 240.0, 240.0, 20.0);
    let rect_stroke_color = Color::new([0.9804, 0.702, 0.5294, 1.]);
    scene.stroke(&stroke, Affine::IDENTITY, rect_stroke_color, None, &rect);

    // Draw a filled circle
    let circle = Circle::new((420.0, 200.0), 120.0);
    let circle_fill_color = Color::new([0.9529, 0.5451, 0.6588, 1.]);
    scene.fill(
        vello::peniko::Fill::NonZero,
        Affine::IDENTITY,
        circle_fill_color,
        None,
        &circle,
    );

    // Draw a filled ellipse
    let ellipse = Ellipse::new((250.0, 420.0), (100.0, 160.0), -90.0);
    let ellipse_fill_color = Color::new([0.7961, 0.651, 0.9686, 1.]);
    scene.fill(
        vello::peniko::Fill::NonZero,
        Affine::IDENTITY,
        ellipse_fill_color,
        None,
        &ellipse,
    );

    // Draw a straight line
    let line = Line::new((260.0, 20.0), (620.0, 100.0));
    let line_stroke_color = Color::new([0.5373, 0.7059, 0.9804, 1.]);
    scene.stroke(&stroke, Affine::IDENTITY, line_stroke_color, None, &line);
}
fn get_time()->(u32,u32,u32,u32){
    let now = Local::now();
    let (_,hour) = now.hour12();
    let minute = now.minute();
    let second = now.second();
    let millisecond = now.timestamp_subsec_millis(); // 直接获取毫秒部分
    (hour,minute,second,millisecond)
}
fn time_to_clock_angle(hour:u32,minute:u32,second:u32,millisecond:u32)->(f64,f64,f64){
    let hour_angle= (hour as f64 % 12.0 + minute as f64 / 60.0 + second as f64 / 3600.0) * 30.0;
    let minute_angle= (minute as f64 + second as f64 / 60.0 + millisecond as f64 / 60000.0) * 6.0;
    let second_angle= (second as f64 + millisecond as f64 / 1000.0) * 6.0;
    (hour_angle,minute_angle,second_angle)
}

fn main()->anyhow::Result<()>{

    let (hour, minute, second, millisecond) = get_time();
    let (hour_angle, minute_angle, second_angle) = time_to_clock_angle(hour, minute, second, millisecond);
    let mut simu= simulation::Simulation::new();
    simu.random_init( 10);
    let res=simu.update();
    println!("res: {:?}", res);
    let res=simu.update();
    println!("res: {:?}", res);
    for i in 0..60{
        let res=simu.update();
        println!("res: {:?}", res);


    }


    

    render::run(Box::new(render))?;
    Ok(())
}