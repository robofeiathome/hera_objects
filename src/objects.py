#!/usr/bin/env python3

# Author: Brubru

import rospy
import math
import tf

from hera_objects.msg import ObjectPosition, ObjectPositionArray, DicBoxes
from hera_objects.srv import FindObject, FindSpecificObject

class Objects:

    def __init__(self):
        self._objects = list()
        self._positions = dict()
        self._obj = None
        rospy.Subscriber('/detector_2d_node/boxes_coordinates', DicBoxes, self.get_detected_objects)

        rospy.Service('objects', FindObject, self.handler)
        rospy.Service('specific_object', FindSpecificObject, self.specific_handler)

        self.listener = tf.TransformListener()
        self.reference_frame = '/manip_base_link'
        rospy.loginfo("[Objects] Dear Operator, I'm ready to give you object coordinates")

    def get_detected_objects(self, array):
        #rospy.loginfo(array)
        #rospy.loginfo(array.detected_objects)
        if not len(array.detected_objects) == 0:
            self._objects.clear()
            for detection in array.detected_objects:
                self._objects.append((detection.type.data, detection.tf_id.data)) # adiciona um novo objeto a lista de objetos

    def get_positions(self, reference=None):
        if reference is None or reference == "":
            reference = self.reference_frame
        print(reference)
        self._positions.clear()
        self._specific = {0: [0.0, 0.0, 0.0]}
        for obj_class, obj_frame in self._objects: # para cada objeto da lista de objetos
            if not obj_frame == '': # se o frame do objeto não for vazio
                try: # tenta obter a posição do objeto
                    trans, a = self.listener.lookupTransform(reference, obj_frame, rospy.Time(0))
                    self._positions[obj_frame] = [trans, obj_class]

                except Exception as e:
                    rospy.loginfo("[Objects] vish!")
                    print(e)
                    self._specific = {0: [0.0, 0.0, 0.0]}
            else:
                # retorna as posicoes zeradas
                self._objects.clear()
                self._positions[obj_frame] = [0.0, 0.0, 0.0]
                self._obj = None
                self._specific = {0: [0.0, 0.0, 0.0]}

    def handler(self, request):
        condition = request.condition.lower()
        succeeded = False
        self.get_positions(request.reference)
        print(self._positions)
        print(request.upper_limit)
        print(request.lower_limit)

        self._coordinates = []
        self._taken_object = []

        if condition == 'closest':
            if request.upper_limit != 0.0 or request.lower_limit != 0.0:
                rospy.loginfo("closest with limits")
                rospy.loginfo(self._positions)
                dist = float("inf")
                for obj_id in self._positions:
                    print(obj_id)
                    x, y, z = self._positions[obj_id][0]
                    trans, a = self.listener.lookupTransform("map", obj_id, rospy.Time(0))
                    print(f"map a = {trans}")
                    if trans[2] > request.lower_limit and trans[2] < request.upper_limit:
                        value = math.sqrt(x**2 + y**2 + z**2)
                        if value < dist:
                            dist = value
                            self._obj = obj_id
                    else:
                        continue
            else:
                rospy.loginfo("closest")
                rospy.loginfo(self._positions)
                dist = float("inf")
                for obj_id in self._positions:
                    print(obj_id)
                    x, y, z = self._positions[obj_id][0]
                    value = math.sqrt(x**2 + y**2 + z**2)
                    if value < dist:
                        dist = value
                        self._obj = obj_id
            
            if self._obj is not None and self._obj in self._positions:
                x, y, z = self._positions[self._obj][0]
                obj_string = self._obj.strip('/').split('/')[1]
                aux = ObjectPosition()
                aux.x = x
                aux.y = y
                aux.z = z
                aux.rx = 0.0
                aux.ry = 0.0
                aux.rz = math.atan2(y,x)
                self._coordinates.append(aux)
                self._taken_object.append(obj_string)
                succeeded = True

            else:
                aux = ObjectPosition()
                aux.x = 0.0
                aux.y = 0.0
                aux.z = 0.0
                aux.rx = 0.0
                aux.ry = 0.0
                aux.rz = 0.0
                self._coordinates.append(aux)
                self._taken_object.append('')

        elif condition == "all":
            rospy.loginfo("all")
            rospy.loginfo(self._positions)
            for obj_id in self._positions:
                x, y, z = self._positions[obj_id][0]
                self._obj = obj_id

                if self._obj is not None and self._obj in self._positions:
                    x, y, z = self._positions[self._obj][0]
                    obj_string = obj_id.strip('/').split('/')[1]
                    aux = ObjectPosition()
                    aux.x = x
                    aux.y = y
                    aux.z = z
                    aux.rx = 0.0
                    aux.ry = 0.0
                    aux.rz = math.atan2(y,x)
                    self._coordinates.append(aux)
                    self._taken_object.append(obj_string)
                    succeeded = True

                else:
                    aux = ObjectPosition()
                    aux.x = 0.0
                    aux.y = 0.0
                    aux.z = 0.0
                    aux.rx = 0.0
                    aux.ry = 0.0
                    aux.rz = 0.0
                    self._coordinates.append(aux)
                    self._taken_object.append('')
        else:
            aux = ObjectPosition()
            aux.x = 0.0
            aux.y = 0.0
            aux.z = 0.0
            aux.rx = 0.0
            aux.ry = 0.0
            aux.rz = 0.0
            self._coordinates.append(aux)
            self._taken_object.append('')

        rospy.loginfo('Found the coordinates!') if succeeded else rospy.loginfo("I'm a shame. Sorry!")
        rospy.loginfo(self._positions)
        return self._coordinates, self._taken_object
    
    def specific_handler(self, request):
        self._coordinates = ObjectPosition()
        obj = request.type
        succeeded = False
        detected_obj = None
 
        #rospy.loginfo(self._specific[0])
        self.get_positions()
        print(self._positions)
        
        for key, value in self._positions.items():
            print("value:", value[1])
            print("obj", obj)
            if value[1] == obj[:-1]:
                detected_obj = value
                break

        if detected_obj:
            rospy.loginfo("object found")
            self._specific = detected_obj

        if self._specific is not None:
            x, y, z = self._specific[0]
            self._coordinates.x = x
            self._coordinates.y = y
            self._coordinates.z = z
            self._coordinates.rx = 0.0
            self._coordinates.ry = 0.0
            self._coordinates.rz = math.atan2(y,x)
            succeeded = True

        else:
            self._coordinates.x = 0.0
            self._coordinates.y = 0.0
            self._coordinates.z = 0.0
            self._coordinates.rx = 0.0
            self._coordinates.ry = 0.0
            self._coordinates.rz = 0.0
            succeeded = True

        rospy.loginfo('Found the coordinates!') if succeeded else rospy.loginfo("I'm a shame. Sorry!")

        return self._coordinates

if __name__ == '__main__':
    rospy.init_node('objects', log_level=rospy.ERROR)
    Objects()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass