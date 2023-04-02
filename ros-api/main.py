from fastapi import FastAPI
from pydantic import BaseModel

class Teleop(BaseModel):
    linear_x: float
    linear_y: float
    linear_z: float
    angular_x: float
    angular_y: float
    angular_z: float

app = FastAPI()

@app.get("/")
async def root():
    '''The base route of the application.'''
    return {'status': 'alive'}

@app.post("/teleop/control")
async def teleop_control(teleop: Teleop):
    '''The route for the web application to send controls to the robot.'''
    app.state.teleop = teleop
    return app.state.teleop

@app.get("/teleop/robot")
async def teleop_robot():
    '''The route for the robot to receive the instructions from the API.'''
    return app.state.teleop
