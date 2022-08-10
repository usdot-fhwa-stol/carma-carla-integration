
import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla  # pylint: disable=import-error
import argparse

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='IP of the host CARLA Simulator (default: localhost)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port of CARLA Simulator (default: 2000)')
    argparser.add_argument(
        '-m', '--map',
        help='load a new map, use --list to see available maps')
    if len(sys.argv) < 2:
        argparser.print_help()
        return

    args = argparser.parse_args()

    maxConnectionAttempts = 10
    while maxConnectionAttempts > 0:
        isConnected = True
        maxConnectionAttempts = maxConnectionAttempts - 1
        try:
            client = carla.Client(args.host, args.port)
            client.set_timeout(5.0)
            world = client.get_world()
        except:
            print("Connecting to CARLA host %r:%r" % (args.host, args.port))
            isConnected=False

        if isConnected:
            print("CARLA simulator connected")
            break
        if maxConnectionAttempts == 0 and isConnected == False:
            print("Maximum connection attempts reached. Check the CARLA server is launched and host/port")
    world = client.load_world(args.map)
main()

