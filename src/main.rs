#![cfg_attr(windows, windows_subsystem = "windows")]
fn main() -> anyhow::Result<()> {
    #[cfg(not(target_os = "android"))]
    {
        use winit::event_loop::EventLoop;
        let event_loop = EventLoop::new()?;

        clocky::run_app(event_loop)?;
        return Ok(())
    }
    #[cfg(target_os = "android")]
    unreachable!();
}



