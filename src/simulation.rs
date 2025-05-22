use nalgebra::{Vector2, vector};
use rapier2d::prelude::{
    BroadPhase, CCDSolver, ColliderBuilder, ColliderSet, DefaultBroadPhase, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet, NarrowPhase, PhysicsPipeline, RigidBodyBuilder, RigidBodyHandle, RigidBodySet
};

const BOUNCE: f32 = 0.8;
pub struct Simulation {
    ballset: Vec<(RigidBodyHandle,f32)>,
    size: (u32, u32),

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
}
impl Simulation {
    pub fn new() -> Simulation {
        Simulation {
            ballset: Vec::new(),
            size: (800, 600),
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
        }
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
        self.ballset.push((handle,ball.0));
    }
    fn out_ball_for_render(&self) -> Vec<(f32, f32,f32)> {
        self.ballset.iter().filter_map(|(handle,radius)|{
            let body=self.rigidbody_set.get(*handle)?;
            let pos=body.translation();
            Some((pos.x,pos.y,*radius))
        }).collect()
    }
    pub fn random_init(&mut self, ball_num: u32) {
        use rand::Rng;
        let mut rng = rand::rng();
        for _ in 0..ball_num {
            let x = rng.random_range(0.0..self.size.0 as f32);
            let y = rng.random_range(0.0..self.size.1 as f32);
            let radius = rng.random_range(-1.0..1.0);
            self.add_ball((radius, Vector2::new(x, y)));
        }
    }

    pub fn update(&mut self) -> Vec<(f32, f32, f32)> {
        self.pipeline.step(
            &vector![0.0, -9.81],
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
