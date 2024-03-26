#!/usr/bin/env python3
import chess
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from chess_robot_service.srv import chess_ai, motion_planning
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
# from chess_robot_msgs.msg import PoseList

class ChessPoseConverter:
    def __init__(self):
        rospy.init_node('chess_pose_converter_node', anonymous=True)
        # connect to chess ai service
        self.ai_service = self.init_service('chess_ai_service', chess_ai)
        # connect to robot service
        self.robot_service = self.init_service('robot_service', motion_planning)
        self.subscriber = rospy.Subscriber('chess_notation_topic', String, self.chess_notation_callback)
        self.chess_board_state_home="rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
        self.started = False
        self.board=chess.Board()
        self.white_capture_count=0
        self.black_capture_count=0
        # self.publisher = rospy.Publisher('pose_topic', PoseArray, queue_size=10)

    def init_service(self, service_name, service_message):
        rospy.wait_for_service(service_name)
        service = rospy.ServiceProxy(service_name, service_message)
        rospy.loginfo('Successfully connected to {}'.format(service_name))
        return service


    def chess_notation_callback(self, msg):
        if msg.data == "start" and not self.started:
            self.started = True
            self.process_next_move()
        else:
            self.send_poses_to_robot(msg.data)
            self.started = False
        # elif self.started:
        #     self.process_feedback(msg.data)


    def process_next_move(self):
        # Send current board state to chess AI service to get the next move
        move_command = self.ai_service(self.chess_board_state_home).command
        # Convert move command to robot poses and send to robot service
        self.send_poses_to_robot(move_command)
        
    def send_poses_to_robot(self, move_command):
        # Convert move command to robot poses and send to robot service
        
        command_sub_parts=move_command.split(',')
        steps, capturing, castling = command_sub_parts[:3]
        if len(command_sub_parts) != 4 and capturing =="no" and castling =="no":
        # robot_pose_list= PoseList()
            robot_pose=Pose()
            robot_pose_list=PoseArray()
            robot_pose_list.header.frame_id = "pedestal"  # Assuming your robot's frame ID is "base_link"
            robot_pose_list.header.stamp = rospy.Time.now()

            
            # Assuming you have a function to convert chess notation to pose quaternion
            # For demonstration purpose, let's say we have a dummy function called "convert_chess_to_pose"
            # chess_notation=msg.data
            # uci_move= self.convert_chess_notation(chess_notation)
            move_1,move_2=self.parse_string_into_halves(steps)
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
            a = ''.join(move_1)  # Concatenate the file and rank into a single string
            chess_piece = self.board.piece_at(chess.parse_square(a))
            move_details = chess_piece.symbol() + "," + capturing + "," + castling
            # Wait for feedback from robot service
            outcome = self.robot_service(robot_pose_list, move_details).feedback
            # Upon receiving feedback, update board state and call process_next_move
            if (outcome==1)&(self.started==True):
                print(self.board)
                self.board.push(chess.Move.from_uci(steps))
                self.chess_board_state_home=self.board.fen()
                print(self.chess_board_state_home)
                self.process_next_move()
        if len(command_sub_parts) != 4 and capturing =="yes" and castling =="no":
        # robot_pose_list= PoseList()
            robot_pose=Pose()
            robot_pose_list=PoseArray()
            robot_pose_list.header.frame_id = "pedestal"  # Assuming your robot's frame ID is "base_link"
            robot_pose_list.header.stamp = rospy.Time.now()

            
            # Assuming you have a function to convert chess notation to pose quaternion
            # For demonstration purpose, let's say we have a dummy function called "convert_chess_to_pose"
            # chess_notation=msg.data
            # uci_move= self.convert_chess_notation(chess_notation)
            move_3,move_4=self.parse_string_into_halves(steps)
            move_1=move_4
            print(move_4)
            a = ''.join(move_1)  # Concatenate the file and rank into a single string
            # print(a)
            chess_piece_1 = self.board.piece_at(chess.parse_square(a))
            # print(chess_piece_1)
            chess_piece=chess_piece_1.symbol()
            
            if(chess_piece.isupper()):
                self.white_capture_count=+1
                move_2=self.get_coordinate(self.white_capture_count,chess_piece)                
            else:
                 self.black_capture_count=+1
                 move_2=self.get_coordinate(self.black_capture_count,chess_piece)    
            moves_capture=[move_1,move_2]
            moves_movement=[move_3,move_4]
            pose_list=[]

            for i in range(2):
                robot_pose = self.convert_chess_to_pose(moves_capture[i])
                # # Set the frame ID for the pose
                # robot_pose.header.frame_id = "pedestal"  # Assuming your robot's frame ID is "base_link"
                # robot_pose.header.stamp = rospy.Time.now()
                # Publish the converted pose
                # robot_pose_list.poses.append(robot_pose)
                pose_list.append(robot_pose)
                
            # a = ''.join(move_1)  # Concatenate the file and rank into a single string
            # chess_piece = self.board.piece_at(chess.parse_square(a)).symbol()
                
            robot_pose_list.poses=pose_list

            move_details = chess_piece + "," + capturing + "," + castling
            # Wait for feedback from robot service
            outcome_1 = self.robot_service(robot_pose_list, move_details).feedback
            
            pose_list=[]

            for i in range(2):
                robot_pose = self.convert_chess_to_pose(moves_movement[i])
                # # Set the frame ID for the pose
                # robot_pose.header.frame_id = "pedestal"  # Assuming your robot's frame ID is "base_link"
                # robot_pose.header.stamp = rospy.Time.now()
                # Publish the converted pose
                # robot_pose_list.poses.append(robot_pose)
                pose_list.append(robot_pose)
                
            robot_pose_list.poses=pose_list
            a = ''.join(move_3)  # Concatenate the file and rank into a single string
            chess_piece_1 = self.board.piece_at(chess.parse_square(a))
            chess_piece=chess_piece_1.symbol()

            move_details = chess_piece + "," + capturing + "," + castling
            # Wait for feedback from robot service
            outcome_2 = self.robot_service(robot_pose_list, move_details).feedback
            # Upon receiving feedback, update board state and call process_next_move
            if (outcome_1==1)&(outcome_2==1)&(self.started==True):
                print(self.board)
                self.board.push(chess.Move.from_uci(steps))
                self.chess_board_state_home=self.board.fen()
                print(self.chess_board_state_home)
                self.process_next_move()
        if len(command_sub_parts) != 4 and capturing =="no" and castling =="yes":
        # robot_pose_list= PoseList()
            robot_pose=Pose()
            robot_pose_list=PoseArray()
            robot_pose_list.header.frame_id = "pedestal"  # Assuming your robot's frame ID is "base_link"
            robot_pose_list.header.stamp = rospy.Time.now()

            
            # Assuming you have a function to convert chess notation to pose quaternion
            # For demonstration purpose, let's say we have a dummy function called "convert_chess_to_pose"
            # chess_notation=msg.data
            # uci_move= self.convert_chess_notation(chess_notation)
            steps_complement=self.get_complement(steps)
            move_1,move_2=self.parse_string_into_halves(steps)
            move_3,move_4=self.parse_string_into_halves(steps_complement)
            print(move_4)
            a = ''.join(move_1)  # Concatenate the file and rank into a single string
            # print(a)
            chess_piece_1 = self.board.piece_at(chess.parse_square(a))
            # print(chess_piece_1)
            chess_piece=chess_piece_1.symbol()
              
            moves_start=[move_1,move_2]
            print()
            moves_end=[move_3,move_4]
            pose_list=[]

            for i in range(2):
                robot_pose = self.convert_chess_to_pose(moves_start[i])
                # # Set the frame ID for the pose
                # robot_pose.header.frame_id = "pedestal"  # Assuming your robot's frame ID is "base_link"
                # robot_pose.header.stamp = rospy.Time.now()
                # Publish the converted pose
                # robot_pose_list.poses.append(robot_pose)
                pose_list.append(robot_pose)
                
            # a = ''.join(move_1)  # Concatenate the file and rank into a single string
            # chess_piece = self.board.piece_at(chess.parse_square(a)).symbol()
                
            robot_pose_list.poses=pose_list

            move_details = chess_piece + "," + capturing + "," + castling
            # Wait for feedback from robot service
            outcome_1 = self.robot_service(robot_pose_list, move_details).feedback
            
            pose_list=[]

            for i in range(2):
                robot_pose = self.convert_chess_to_pose(moves_end[i])
                # # Set the frame ID for the pose
                # robot_pose.header.frame_id = "pedestal"  # Assuming your robot's frame ID is "base_link"
                # robot_pose.header.stamp = rospy.Time.now()
                # Publish the converted pose
                # robot_pose_list.poses.append(robot_pose)
                pose_list.append(robot_pose)
                
            robot_pose_list.poses=pose_list
            a = ''.join(move_3)  # Concatenate the file and rank into a single string
            chess_piece_1 = self.board.piece_at(chess.parse_square(a))
            chess_piece=chess_piece_1.symbol()

            move_details = chess_piece + "," + capturing + "," + castling
            # Wait for feedback from robot service
            outcome_2 = self.robot_service(robot_pose_list, move_details).feedback
            # Upon receiving feedback, update board state and call process_next_move
            if (outcome_1==1)&(outcome_2==1)&(self.started==True):
                print(self.board)
                self.board.push(chess.Move.from_uci(steps))
                self.chess_board_state_home=self.board.fen()
                print(self.chess_board_state_home)
                self.process_next_move()
        


    def convert_chess_to_pose(self, chess_notation):
        cross_reference_dict = {}
        # Populate the dictionary with the appropriate value in numbers for example b is 1
        for i, letter in enumerate('abcdefghklij', start=0):
            cross_reference_dict[letter] = str(i)
        cross_reference_dict["i"]=-2
        cross_reference_dict["j"]=-1
        unit_square = 0.0375
        robot_pose = Pose()
        robot_pose.position.x=0.020 - unit_square*4
        robot_pose.position.y=0.4185 - unit_square
        robot_pose.position.z=0.22
        robot_pose.orientation.x=0.00212
        robot_pose.orientation.y=0.99998
        robot_pose.orientation.z=-0.0059781
        robot_pose.orientation.w=0.000400
        
        # delta_x_alpha,delta_y=self.parse_string_into_halves(chess_notation)
        delta_x_alpha=chess_notation[0]
        delta_y=chess_notation[1]
        delta_x=cross_reference_dict.get(delta_x_alpha)
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
    
    def get_coordinate(self, count, chess_piece):
        if(chess_piece.isupper()):
            if(count<=8):
                final_coordinates= "j" + str(count)
            else:
                final_coordinates= "i" + str(count-8)
        else:
            if(count<=8):
                final_coordinates= "k" + str(count)
            else:
                final_coordinates= "l" + str(count-8)
        print(final_coordinates)
        return final_coordinates
        
    def get_complement(self, steps):
       my_dict = {'e1g1': 'h1f1', 'e1c1': 'a1d1', 'e8g8': 'h8f8', 'e1c1':'a8d8'} 
       output=my_dict.get(steps)
       return output

if __name__ == '__main__':
    try:
        converter = ChessPoseConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass