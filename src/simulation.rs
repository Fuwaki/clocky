use nalgebra::{point, vector, Rotation, Vector2};
use rapier2d::{
    math::{Isometry, Point},
    prelude::{
        BroadPhase, CCDSolver, ColliderBuilder, ColliderHandle, ColliderSet, DefaultBroadPhase,
        ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet, NarrowPhase,
        PhysicsPipeline, RigidBodyBuilder, RigidBodyHandle, RigidBodySet, SharedShape,
    },
};
use rayon::prelude::*;
fn generate_circle_points(radius: f32, segments: usize) -> Vec<Point<f32>> {
    let mut points = Vec::with_capacity(segments + 1);
    let angle_step = 2.0 * std::f32::consts::PI / segments as f32;

    for i in 0..segments {
        let angle = angle_step * i as f32;
        points.push(point![radius * angle.cos(), radius * angle.sin()]);
    }

    if let Some(&first) = points.first() {
        points.push(first); // 闭合多边形
    }

    points
}
const BOUNCE: f32 = 0.8;
pub struct Simulation {
    ballset: Vec<(RigidBodyHandle, f32)>,
    size: (u32, u32),
    hour: Option<(RigidBodyHandle, ColliderHandle)>,
    second: Option<(RigidBodyHandle, ColliderHandle)>,
    minute: Option<(RigidBodyHandle, ColliderHandle)>,

    pipeline: PhysicsPipeline,
    rigidbody_set: RigidBodySet,
    collider_set: ColliderSet,
    integration_parameters: IntegrationParameters,
    islands: IslandManager,
    broad_phase: Box<dyn BroadPhase>,
    narrow_phase: NarrowPhase,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    boundary_rigid_body: Option<RigidBodyHandle>,
    boundary_collider: Option<ColliderHandle>,
}
impl Simulation {
    pub fn new() -> Simulation {
        let mut s = Simulation {
            ballset: Vec::new(),
            size: (768, 1034),
            //然后就是物理引擎的一堆东西
            pipeline: PhysicsPipeline::new(),
            rigidbody_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            integration_parameters: IntegrationParameters::default(),
            islands: IslandManager::new(),
            broad_phase: Box::new(DefaultBroadPhase::new()),
            narrow_phase: NarrowPhase::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            //边界
            boundary_rigid_body: None,
            boundary_collider: None,
            hour: None,
            second: None,
            minute: None,
        };
        s
    }
    pub fn add_ball(&mut self, ball: (f32, Vector2<f32>)) {
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![ball.1.x, ball.1.y])
            .build();
        let handle = self.rigidbody_set.insert(rigid_body);
        let collider = ColliderBuilder::ball(ball.0)
            .restitution(BOUNCE)
            .density(1.0)
            .build();
        //TODO:根据大小计算密度
        self.collider_set
            .insert_with_parent(collider, handle, &mut self.rigidbody_set);
        self.ballset.push((handle, ball.0));
    }
    fn add_boundary(&mut self) {
        let points = generate_circle_points(self.size.1 as f32 / 2.5, 64);
        let shape = SharedShape::polyline(points, None);
        let collider = ColliderBuilder::new(shape)
            .restitution(BOUNCE)
            .density(1.0)
            .build();
        let rigid_body = RigidBodyBuilder::fixed()
            .translation(vector![self.size.0 as f32 / 2.0, self.size.1 as f32 / 2.0])
            .build();
        let rb_handle = self.rigidbody_set.insert(rigid_body);
        let collider_handle =
            self.collider_set
                .insert_with_parent(collider, rb_handle, &mut self.rigidbody_set);
        self.boundary_rigid_body = Some(rb_handle);
        self.boundary_collider = Some(collider_handle);
    }
    fn rebuild_boundary(&mut self) {
        if let Some(collider) = self.boundary_collider {
            self.collider_set
                .remove(collider, &mut self.islands, &mut self.rigidbody_set, false);
        }
        if let Some(rb_handle) = self.boundary_rigid_body {
            self.rigidbody_set.remove(
                rb_handle,
                &mut self.islands,
                &mut self.collider_set,
                &mut self.impulse_joints,
                &mut self.multibody_joints,
                true,
            );
        }

        self.add_boundary();
    }
    fn add_clock(&mut self) {
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![self.size.0 as f32 / 2.0, self.size.1 as f32 / 2.0])
            .lock_translations()
            .build();
        let collider_offset=Isometry::translation(0.0, 0.0);
        const hour_length: f32 = 400.0;
        const hour_width: f32 = 10.0;
        let collider=ColliderBuilder::cuboid(hour_length, hour_width).density(1.0).position(collider_offset).build();
        let rb_handle = self.rigidbody_set.insert(rigid_body);
        let collider_handle =
            self.collider_set
                .insert_with_parent(collider, rb_handle, &mut self.rigidbody_set);
        self.hour = Some((rb_handle, collider_handle));
        // let collider=ColliderBuilder::cuboid(hx, hy)
    }
    fn rebuild_clock(&mut self) {
        if let Some(collider) = self.hour {
            self.collider_set
                .remove(collider.1, &mut self.islands, &mut self.rigidbody_set, false);
        }
        if let Some(rb_handle) = self.hour {
            self.rigidbody_set.remove(
                rb_handle.0,
                &mut self.islands,
                &mut self.collider_set,
                &mut self.impulse_joints,
                &mut self.multibody_joints,
                true,
            );
        }
        self.add_clock();
    }
    pub fn set_clock_angle(&mut self,angle:(f32,f32,f32)){
        if let Some((handle, _)) = self.hour {
            let body = self.rigidbody_set.get_mut(handle).unwrap();
            let rotation = nalgebra::UnitComplex::new(angle.0);
            body.set_angvel(1.0, true);
            println!("angle: {:?}", angle);
            // body.set_translation(vector![angle.1, angle.2]);
            
        }
    }
    

    pub fn on_resize(&mut self, size: (u32, u32)) {
        self.size = size;
        self.rebuild_boundary();
        self.rebuild_clock();
        self.random_init(4000);
        self.cleanup_out_of_bounds();
    }
    fn out_ball_for_render(&self) -> Vec<(f32, f32, f32)> {
        self.ballset
            .iter()
            .filter_map(|(handle, radius)| {
                let body = self.rigidbody_set.get(*handle)?;
                let pos = body.translation();
                Some((pos.x, pos.y, *radius))
            })
            .collect()
    }
    pub fn random_init(&mut self, ball_num: u32) {
        use rand::Rng;
        let mut rng = rand::rng();
        for _ in 0..ball_num {
            let x = rng.random_range(0.0..self.size.0 as f32);
            let y = rng.random_range(0.0..self.size.1 as f32);
            let radius = rng.random_range(1.0..10.0);
            self.add_ball((radius, Vector2::new(x, y)));
        }
        self.cleanup_out_of_bounds();
    }
    fn cleanup_out_of_bounds(&mut self) {
        use rayon::prelude::*;

        let indices_to_remove: Vec<usize> = self
            .ballset
            .par_iter()
            .enumerate()
            .filter_map(|(i, (handle, _))| {
                let body = self.rigidbody_set.get(*handle)?;
                let pos = body.translation();
                if (pos.x - self.size.0 as f32 / 2.0).powi(2)
                    + (pos.y - self.size.1 as f32 / 2.0).powi(2)
                    > (self.size.1 as f32 / 2.5).powi(2)
                {
                    Some(i)
                } else {
                    None
                }
            })
            .collect();

        for i in indices_to_remove.iter().rev() {
            let (handle, _) = self.ballset.remove(*i);
            self.rigidbody_set.remove(
                handle,
                &mut self.islands,
                &mut self.collider_set,
                &mut self.impulse_joints,
                &mut self.multibody_joints,
                true,
            );
        }
    }

    pub fn update(&mut self) -> Vec<(f32, f32, f32)> {
        self.pipeline.step(
            &vector![0.0, 9.81],
            &self.integration_parameters,
            &mut self.islands,
            &mut *self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigidbody_set,
            &mut self.collider_set,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd_solver,
            None,
            &(),
            &(),
        );

        return self.out_ball_for_render();
    }
}
