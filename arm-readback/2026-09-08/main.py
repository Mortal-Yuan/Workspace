"""Car-commanded grab endpoint; startup does not move any joint.

Factory manual application remains available as factory.z_main.z_main().
Never run it concurrently with this service because both own UART2.
"""
from factory.z_grab_service import serve

if __name__ == '__main__':
    serve()
