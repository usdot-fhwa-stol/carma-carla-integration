# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.
import carla
import logging
import argparse

def wait_for_carla(args):
    """
    This is a simple python script to standup a CARLA client that only terminates onces a successful connection to CARLA is established. This avoids timeout issues with the CARLA Ros Bridge described here (https://github.com/usdot-fhwa-stol/carma-carla-integration/issues/65). This script can simply be called as a precursor to CARLA depedent services to ensure that CARLA is healthy before launching.
    """
    connected = False
    
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
logging.getLogger().setLevel(logging.INFO)
arg_parser = argparse.ArgumentParser(description=wait_for_carla.__doc__)
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
wait_for_carla(args)