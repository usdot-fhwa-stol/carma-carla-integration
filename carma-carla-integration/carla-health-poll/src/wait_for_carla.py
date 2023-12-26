import carla
import logging
logging.getLogger().setLevel(logging.INFO)
connected = False
while not connected:
    try:
        logging.info("Attempting to connect to CARLA at 172.2.0.2:2000")
        carla_client = carla.Client(
                "172.2.0.2",
                2000)
        carla_client.set_timeout(10)
        carla_world = carla_client.get_world()
        connected = True
    except (IOError, RuntimeError) as e:
        logging.warning("Error connecting to carla: {}".format(e))
    except KeyboardInterrupt:
        pass
logging.info("CARLA Connection successful ... ")