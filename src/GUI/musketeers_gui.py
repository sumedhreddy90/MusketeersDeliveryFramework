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
   
    def _init_(self, **kwargs):
        


    def set_robot(self, robotlist):
        
    
    def set_delivery(self, delivery_type):
       
                
    
    def set_location(self, location_list):
       
    
    def start_mission(self,*args):
        
        
    def build(self):
        

if _name_ == '_main_':
    
    
    
    MyApp = MusketeersApp()
    MyApp.run()