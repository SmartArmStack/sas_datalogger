import rospy
from rosilo_datalogger.msg import AddValueMsg
from std_msgs.msg import Float64MultiArray


class DataloggerInterface:
    def __init__(self, queue_size):
        # sc_save = node_handle.serviceClient < rosilo_datalogger::Save > ("/rosilo_datalogger/save");
        self.publisher_add_value_ = rospy.Publisher("set/desired_pose",
                                                    AddValueMsg,
                                                    queue_size=queue_size)

    def log(self, name, value):
        msg = AddValueMsg()
        msg.name = name
        if type(value) is str:
            strvalue = value
        elif type(value) is float or type(value) is int:
            msg.value = Float64MultiArray(value)
        else:
            # If the array is (x, 1)
            # If the array is (1, y)
            try:
                (x, y) = value.shape
                msg.value = value.reshape(max(x,y))
            except ValueError as e:
                pass
            # If the array is (x,)
            try:
                (x,) = value.shape
                msg.value = value
            except ValueError as e:
                pass

            raise Exception("DataloggerInterface::Unknown value type={}.".format(type(value)))

        self.publisher_add_value_.publish(msg)