import rospy
import select, socket 
from std_msgs.msg import Bool
from SocketServer import UDPServer, BaseRequestHandler
from threading import Thread
import argparse

''' 
author    t.knapp
version   20150624-1
'''

'''
Schnittstelle zwischen Schiedsrichter-App und ROS-Welt.

Horcht auf UDP-Port 4711 (ACHTUNG FIREWALL!)

Publiziert Bool auf 'soccer/referee'
- True  : Start/Spiel läuft
- False : Stopp/Spiel unterbrechen

CMDLINE:
-v   verbose mode

Entwickelt von Team Böing, weitergegeben an Team Grunwald
'''
class Handler(BaseRequestHandler):
    
    def handle(self):
        #print "message:", self.request[0]
        #print "from:", self.client_address
        self.server.callback(self.request[0], self.client_address)

class NodeReferee(object):
    
    def __init__(self, verbose=False, name="NodeReferee"):
        rospy.init_node(name, anonymous=False)
        rospy.loginfo("Stop referee by pressing CTRL + C")
        
        self.verbose = verbose
        if verbose:
            rospy.loginfo("Being verbose")
        rospy.on_shutdown(self.shutdown)

        # Publisher 
        self.topic = rospy.Publisher('soccer/referee', Bool, queue_size=10)

        # UDP Server with callback
        addr = ("", 4711)
        rospy.loginfo("listening on %s:%s" % addr)
        self.server = UDPServer(addr, Handler)
        self.server.timeout = 5
        self.server.callback = self.publish
        
        # Stoppable serve_forever workaround =)
        self.run = True
        while(self.run):
            self.server.handle_request()
        
    def publish(self, data, client_address):
        # By definition:
        # 1 => keep going / start
        # 0 => stop
        running = (data == "1")
        if self.verbose:
            rospy.loginfo("Received '%s' from %s considered as %s" % (data, client_address, str(running)))
        self.topic.publish(running)
    
    def shutdown(self):
        rospy.loginfo("Shutting down ... (can take up to %d seconds)" % self.server.timeout)
        self.run = False
           
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', help='Be verbose', default=False, action='store_true')
    args = parser.parse_args()

    NodeReferee(verbose=args.verbose)

