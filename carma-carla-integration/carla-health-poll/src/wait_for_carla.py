import carla
import logging
import argparse

logging.getLogger().setLevel(logging.INFO)
connected = False
arg_parser = argparse.ArgumentParser(description=__doc__)
arg_parser.add_argument(
        "--carla-host",
        default="172.2.0.2",
        type=str,
        help="CARLA host. (default: \"localhost\")")

arg_parser.add_argument(
    "--carla-port",
    default=2000,
    type=int,
    help="CARLA host. (default: 2000)")
arg_parser.add_argument(
    "--carla-timeout",
    default=10,
    type=int,
    help="CARLA Client timeout in seconds. (default: 10)")
args = arg_parser.parse_args()

while not connected:
    try:
        logging.info(f"Attempting to connect to CARLA at {args.carla_host}:{args.carla_port}" )
        carla_client = carla.Client(
                args.carla_host,
                args.carla_port)
        carla_client.set_timeout(args.carla_timeout)
        carla_world = carla_client.get_world()
        connected = True
    except (IOError, RuntimeError) as e:
        logging.warning("Error connecting to carla: {}".format(e))
    except KeyboardInterrupt:
        pass
logging.info("CARLA Connection successful ... ")