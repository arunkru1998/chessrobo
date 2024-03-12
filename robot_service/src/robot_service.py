#!/usr/bin/env python3
import chess
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
# from chess_robot_msgs.msg import PoseList

class ChessPoseConverter:
    def __init__(self):
        rospy.init_node('chess_pose_converter_node', anonymous=True)
        self.subscriber = rospy.Subscriber('chess_notation_topic', String, self.chess_notation_callback)
        self.publisher = rospy.Publisher('pose_topic', PoseArray, queue_size=10)

    def chess_notation_callback(self, msg):
        # Convert chess notation to quaternion pose

        # robot_pose_list= PoseList()
        robot_pose=Pose()
        robot_pose_list=PoseArray()
        robot_pose_list.header.frame_id = "pedestal"  # Assuming your robot's frame ID is "base_link"
        robot_pose_list.header.stamp = rospy.Time.now()

        
        # Assuming you have a function to convert chess notation to pose quaternion
        # For demonstration purpose, let's say we have a dummy function called "convert_chess_to_pose"
        chess_notation=msg.data
        uci_move= self.convert_chess_notation(chess_notation)
        move_1,move_2=self.parse_string_into_halves(uci_move)
        moves=[move_1,move_2]
        pose_list=[]

        for i in range(2):
            robot_pose = self.convert_chess_to_pose(moves[i])
            # # Set the frame ID for the pose
            # robot_pose.header.frame_id = "pedestal"  # Assuming your robot's frame ID is "base_link"
            # robot_pose.header.stamp = rospy.Time.now()
            # Publish the converted pose
            # robot_pose_list.poses.append(robot_pose)
            pose_list.append(robot_pose)
            
        robot_pose_list.poses=pose_list
        self.publisher.publish(robot_pose_list)
        rospy.sleep(10)
        


    def convert_chess_to_pose(self, chess_notation):
        cross_reference_dict = {}
        # Populate the dictionary with the appropriate value in numbers for example b is 1
        for i, letter in enumerate('abcdefgh', start=0):
            cross_reference_dict[letter] = str(i)
        unit_square = 0.0375
        robot_pose = Pose()
        robot_pose.position.x=0.020 - unit_square*4
        robot_pose.position.y=0.4185 - unit_square
        robot_pose.position.z=0.28
        robot_pose.orientation.x=0.00212
        robot_pose.orientation.y=0.99998
        robot_pose.orientation.z=-0.0059781
        robot_pose.orientation.w=0.000400
        
        delta_x_alpha,delta_y=self.parse_string_into_halves(chess_notation)
        delta_x=cross_reference_dict.get(delta_x_alpha[0])
        robot_pose.position.x+=int(delta_x)*unit_square
        robot_pose.position.y+=(int(delta_y[0])-1)*unit_square

        return robot_pose
    
    def convert_chess_notation(self,chess_notation):
        try:
            # Create an empty chess board
            board = chess.Board()

            # covert the normal algebric input to UCI 
            move = board.push_san(chess_notation).uci()

            return move
        except ValueError:
            return None  # Invalid notation
        

    def parse_string_into_halves(self,input_string):
        # Calculate the midpoint index
        midpoint = len(input_string) // 2
        
        # Split the string into two halves
        first_half = list(input_string[:midpoint])
        second_half = list(input_string[midpoint:])
        
        return first_half, second_half

if __name__ == '__main__':
    try:
        converter = ChessPoseConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass