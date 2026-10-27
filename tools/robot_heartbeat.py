"""Shared heartbeat fields: receiver is independent of the ROS Python GIL."""
class SharedHeartbeat:
    def __init__(self,context,initial):
        self.values={key:context.Value('d' if key=='last' else 'b',value) for key,value in initial.items()}
    def __getitem__(self,key):return self.values[key].value
    def __setitem__(self,key,value):self.values[key].value=value
    def get(self,key,default=None):return self[key] if key in self.values else default
    def update(self,**values):
        for key,value in values.items():self[key]=value
