import rospy
import rosnode

def temp():
    print(rosnode.get_node_names())

if __name__ == "__main__":
    temp()