#!/usr/bin/env python3
from std_srvs.srv import Trigger
from robot_interface_eki.srv import Grip
from robot_interface_eki.srv import Move
import rclpy
from rclpy.node import Node


class Client(Node):

    def __init__(self, base_index, tool_index):
        super().__init__('Client')
        self.base_index = base_index
        self.tool_index = tool_index

    
    def waitresponse(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                try:
                    response = self.future.result()
                except Exception as e:
                    self.get_logger().info('Service call failed %r' % (e,))
                else:
                    self.get_logger().info(response.message) 
                break


    def run(self):      
        self.cli = self.create_client(Trigger, 'run_srv')      
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Trigger.Request()            
        self.future = self.cli.call_async(self.req)
        self.waitresponse()

    def move(self,target=-1, joints=[], cartesian=[], lin=False):
        """ 
        Possibilities:
        -- obj.move(target=id_target)
        -- obj.move(joints=[a1 a2 a3 a4 a5 a6 a7])
        -- obj.move(cartesian=[x y z a b c], lin=True/False)
        """      
        self.cli = self.create_client(Move, 'move_srv')      
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Move.Request()
        self.req.target = target
        self.req.lin = lin
        self.req.a = 0.0
        self.req.b = 0.0
        self.req.c = 0.0
        self.req.x = 0.0
        self.req.y = 0.0
        self.req.z = 0.0
        self.req.a1 = 0.0
        self.req.a2 = 0.0
        self.req.a3 = 0.0 
        self.req.a4 = 0.0 
        self.req.a5 = 0.0 
        self.req.a6 = 0.0 
        self.req.a7 = 0.0 
        self.req.base_index = self.base_index
        self.req.tool_index = self.tool_index  
 	
        if (target != -1):
            self.req.type = self.req.TARGET
        if (joints):
            self.req.type = self.req.JOINTS
            self.req.a1 = joints[0]
            self.req.a2 = joints[1]
            self.req.a3 = joints[2]
            self.req.a4 = joints[3]
            self.req.a5 = joints[4]
            self.req.a6 = joints[5]
            self.req.a7 = joints[6]

        if (cartesian):
            self.req.type = self.req.CARTESIAN
            self.req.x = cartesian[0]
            self.req.y = cartesian[1]
            self.req.z = cartesian[2]
            self.req.a = cartesian[3]
            self.req.b = cartesian[4]
            self.req.c = cartesian[5]     
           
        self.future = self.cli.call_async(self.req)
        self.waitresponse()

    def grip(self,item_size=-1.0, close=False, suction_active=False, cylinder_position=-1.0):      
        """ 
        Possibilities:
        -- obj.grip(item_size=a_num, close=True/False)   JAW
        -- obj.grip(suction_active=True/False, cylinder_position=a_num)   VACUUM
        -- obj.grip(item_size=a_num, close=True/False, suction_active=True/False, cylinder_position=other_num)  COMBINED
        """   
        self.cli = self.create_client(Grip, 'grip_srv')      
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Grip.Request() 
        if ((item_size != -1.0) and (cylinder_position != -1.0)):
            self.req.type = self.req.COMBINED
        elif (item_size != -1.0):
            self.req.type = self.req.JAW
        else:
            self.req.type = self.req.VACUUM
        self.req.item_size = item_size 
        self.req.close = close
        self.req.suction_active = suction_active
        self.req.cylinder_position = cylinder_position
        self.future = self.cli.call_async(self.req)
        self.waitresponse()



def main(args=None):
    rclpy.init(args=args)

    _client = Client(base_index=4,tool_index = 4)
    

########################################### 
#####  Enter the list commands here  ######
###########################################

    _client.move(cartesian=[0.0, 600.0, 800.0, 90.0, 0.0, 0.0], lin=False)
    _client.move(cartesian=[0.0, 400.0, 800.0, 90.0, 0.0, 0.0], lin=True)
    _client.move(cartesian=[0.0, 600.0, 800.0, 90.0, 0.0, 0.0], lin=True)
    #_client.move(cartesian=[114.0, 535.0, 365.0, 0.0, 0.0, 0.0], lin=False)
    #_client.move(cartesian=[93.0, 165.0, 365.0, 0.0, 0.0, 0.0], lin=True)
    # _client.move(target=5)
    # _client.move(joints=[0.2, 1.0, 2.2, 3.4, 1.0, 1.0, 1.0])
    # _client.move(cartesian=[0.2, 1.0, 2.2, 3.4, 1.0, 1.0], lin=False)
    
    # _client.grip(item_size=8.3, close=True)
    # _client.grip(suction_active=True, cylinder_position=1.8)
    # _client.grip(item_size=1.2, close=False, suction_active=False, cylinder_position=3.6)



###########################################
###########################################   

    _client.run()
    _client.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
