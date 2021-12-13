import rospy
from std_msgs.msg import String
from std_msgs.msg import MultiArrayDimension
from re import A
from kivy.lang import Builder
from tf.transformations import quaternion_from_euler
from kivy.metrics import dp
from kivymd.uix.gridlayout import MDGridLayout
from kivy.properties import StringProperty
from kivymd.uix.list import OneLineIconListItem
from kivymd.app import MDApp
from kivymd.uix.menu import MDDropdownMenu
from kivymd.uix.gridlayout import GridLayout
from kivymd.uix.boxlayout import BoxLayout
from kivy.uix.textinput import TextInput
from kivymd.uix.button import BaseButton
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.image import Image
from geometry_msgs.msg import PoseStamped
from kivymd.uix.stacklayout import StackLayout


class MainBox(BoxLayout):
    pass

class IconListItem(OneLineIconListItem):
    icon = StringProperty()

class DisplayScreen(MDGridLayout):
 
    def test(self):
       pass

class MusketeersApp(MDApp):
   
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.screen=Builder.load_file('/home/starfleeet-robotics/808x/src/MusketeersDeliveryFramework/src/GUI/musketeers.kv')
        self.robot_var = "default"
        self.delivery_var = "default"
        self.base_var = "default"
        self.location_var = "default"
        
        menu_items = [
            {
                "viewclass": "IconListItem",
                "icon": "robot",
                "text": f"Base#{i+1}",
                "height": dp(56),
                "on_release": lambda x=f"Base#{i+1}": self.set_item(x),
            } for i in range(4)
        ]
        self.menu = MDDropdownMenu(
            caller=self.screen.ids.base_item,
            items=menu_items,
            width_mult=4,
        )
        self.menu.bind()
        
        delivery_items = [
            {
                "viewclass": "IconListItem",
                "icon": "robot",
                "text": f"Pickup",
                "height": dp(56),
                "on_release": lambda x=f"Pickup": self.set_delivery(x),
            },
            {
                "viewclass": "IconListItem",
                "icon": "robot",
                "text": f"Delivery",
                "height": dp(56),
                "on_release": lambda x=f"Delivery": self.set_delivery(x),
            } 
        ]
        self.delivery_menu = MDDropdownMenu(
            caller=self.screen.ids.delivery_item,
            items=delivery_items,
            position="center",
            width_mult=4,
            )
        self.delivery_menu.bind()

        
        

        
    
    def set_item(self, text_item):
        self.screen.ids.base_item.set_item(text_item)
        self.base_var = text_item
        print(text_item)
        if(text_item == "Base#1"):
            robot_items = [
            {
                "viewclass": "IconListItem",
                "icon": "robot",
                "text": f"Husky#{i+1}",
                "height": dp(56),
                "on_release": lambda x=f"hsk#{i+1}": self.set_robot(x),
            } for i in range(5)
        ]
            self.robot_menu = MDDropdownMenu(
            caller=self.screen.ids.robot_item,
            items=robot_items,
            position="center",
            width_mult=4,
            )
            self.robot_menu.bind()
            #set_robot(mylist)
            # self.screen.ids.robot_item.set_robot(mylist)
        elif(text_item == "Base#2"):
            robot_items = [
            {
                "viewclass": "IconListItem",
                "icon": "robot",
                "text": f"Husky#{i + 6}",
                "height": dp(56),
                "on_release": lambda x=f"hsk#{i + 6}": self.set_robot(x),
            } for i in range(5)
        ]
            self.robot_menu = MDDropdownMenu(
            caller=self.screen.ids.robot_item,
            items=robot_items,
            position="center",
            width_mult=4,
            )
            self.robot_menu.bind()
        elif(text_item == "Base#3"):

            robot_items = [
            {
                "viewclass": "IconListItem",
                "icon": "robot",
                "text": f"Husky#{i + 11}",
                "height": dp(56),
                "on_release": lambda x=f"hsk#{i + 11}": self.set_robot(x),
            } for i in range(5)
        ]
            self.robot_menu = MDDropdownMenu(
            caller=self.screen.ids.robot_item,
            items=robot_items,
            position="center",
            width_mult=4,
            )
            self.robot_menu.bind()

        elif(text_item == "Base#4"):

            robot_items = [
            {
                "viewclass": "IconListItem",
                "icon": "robot",
                "text": f"Husky#{i + 16}",
                "height": dp(56),
                "on_release": lambda x=f"hsk#{i + 16}": self.set_robot(x),
            } for i in range(5)
        ]
            self.robot_menu = MDDropdownMenu(
            caller=self.screen.ids.robot_item,
            items=robot_items,
            position="center",
            width_mult=4,
            )
            self.robot_menu.bind()
        else:
            print("error")    
        self.menu.dismiss()

    def set_robot(self, robotlist):
        print(robotlist)
        self.robot_var = robotlist
        self.screen.ids.robot_item.set_item(robotlist)
        self.menu.dismiss()
        return robotlist
    
    def set_delivery(self, delivery_type):
        print(delivery_type)
        self.delivery_var = delivery_type
        if(delivery_type == "Pickup"):
            location_items = [
            {
                "viewclass": "IconListItem",
                "icon": "robot",
                "text": f"Chipottle",
                "height": dp(56),
                "on_release": lambda x=f"Chipottle": self.set_location(x),
            },
            {
                "viewclass": "IconListItem",
                "icon": "robot",
                "text": f"Subway",
                "height": dp(56),
                "on_release": lambda x=f"Subway": self.set_location(x),
            },
            {
                "viewclass": "IconListItem",
                "icon": "robot",
                "text": f"Taco",
                "height": dp(56),
                "on_release": lambda x=f"Taco": self.set_location(x),
            },
            {
                "viewclass": "IconListItem",
                "icon": "robot",
                "text": f"Panda Express",
                "height": dp(56),
                "on_release": lambda x=f"Panda Express": self.set_location(x),
            } 
        ]
            self.location_menu = MDDropdownMenu(
            caller=self.screen.ids.location_item,
            items=location_items,
            position="center",
            width_mult=4,
            )
            self.location_menu.bind()
            self.screen.ids.delivery_item.set_item(delivery_type)        
        else: 
            location_items = [
            {
                "viewclass": "IconListItem",
                "icon": "robot",
                "text": f"Iribe",
                "height": dp(56),
                "on_release": lambda x=f"Iribe": self.set_location(x),
            },
            {
                "viewclass": "IconListItem",
                "icon": "robot",
                "text": f"JMP",
                "height": dp(56),
                "on_release": lambda x=f"JMP": self.set_location(x),
            },
            {
                "viewclass": "IconListItem",
                "icon": "robot",
                "text": f"Domain",
                "height": dp(56),
                "on_release": lambda x=f"Domain": self.set_location(x),
            },
            {
                "viewclass": "IconListItem",
                "icon": "robot",
                "text": f"McKeldin",
                "height": dp(56),
                "on_release": lambda x=f"McKeldin": self.set_location(x),
            } 
        ]
            self.location_menu = MDDropdownMenu(
            caller=self.screen.ids.location_item,
            items=location_items,
            position="center",
            width_mult=4,
            )
            self.location_menu.bind()
            self.screen.ids.delivery_item.set_item(delivery_type)
    
    def set_location(self, location_list):
        self.location_var = location_list
        self.screen.ids.location_item.set_item(location_list)
        print(location_list)
    
    def start_mission(self,*args):
        print("starting mission")
        rospy.sleep(1)
        checkpoint = PoseStamped()

        checkpoint.pose.position.x = -11.5
        checkpoint.pose.position.y = 10
        checkpoint.pose.position.z = 0.0

        [x,y,z,w]=quaternion_from_euler(0.0,0.0,-3.14)
        checkpoint.pose.orientation.x = x
        checkpoint.pose.orientation.y = y
        checkpoint.pose.orientation.z = z
        checkpoint.pose.orientation.w = w

        checkpoint.header.frame_id = "map"

        checkpoint.header.stamp = rospy.Time.now()
        pub_base.publish(checkpoint)
        print(checkpoint)
        # # pub_base.publish(self.base_var)
        # # pub_base.publish(self.robot_var)
        # # pub_base.publish(self.delivery_var)
        # # pub_base.publish(self.location_var)
        
    def build(self):
        self.theme_cls.theme_style = "Dark"
        return self.screen

if __name__ == '__main__':
    
    
    rospy.init_node('musky_gui', anonymous=True)
    pub_base = rospy.Publisher('/hsk01/move_base_simple/goal',PoseStamped, queue_size=10)
    
    MyApp = MusketeersApp()
    MyApp.run()