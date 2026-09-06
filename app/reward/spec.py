from dataclasses import dataclass, asdict
from app.types import RobotState, MotorCommand
@dataclass
class RewardSpec:
    survival:float=1.; angle:float=3.; angular_velocity:float=.2; angular_acceleration:float=.05; position:float=.1; velocity:float=.05; motor_effort:float=.01; motor_jerk:float=.01; motor_asymmetry:float=.02
    def validate(self):
        for k,v in asdict(self).items():
            if not isinstance(v,(int,float)) or not 0<=v<=1000: raise ValueError(f"Invalid reward {k}")
        return self
    def score(self,s:RobotState, command:MotorCommand, previous:MotorCommand|None=None, alive=True):
        previous=previous or MotorCommand(); a=(command.left+command.right)/2
        return (self.survival if alive else 0)-self.angle*s.pitch**2-self.angular_velocity*s.pitch_rate**2-self.angular_acceleration*s.pitch_accel**2-self.position*s.position**2-self.velocity*s.velocity**2-self.motor_effort*a*a-self.motor_jerk*(a-(previous.left+previous.right)/2)**2-self.motor_asymmetry*(command.left-command.right)**2
