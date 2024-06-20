#!/usr/bin/env python3

import rospy
import math
import tf

from hera_objects.msg import ObjectPosition, ObjectPositionArray, DicBoxes
from hera_objects.srv import FindObject, FindSpecificObject

class Objects:

    def __init__(self):
        self._objects = []
        self._positions = {}
        self._obj = None
        rospy.Subscriber('/detector_2d_node/boxes_coordinates', DicBoxes, self.get_detected_objects)
        rospy.Service('objects', FindObject, self.handler)
        rospy.Service('specific_object', FindSpecificObject, self.specific_handler)
        self.listener = tf.TransformListener()
        self.reference_frame = '/manip_base_link'
        rospy.loginfo("[Objects] Dear Operator, I'm ready to give you object coordinates")

    def get_detected_objects(self, array):
        self._objects = [(detection.type.data, detection.tf_id.data) for detection in array.detected_objects if array.detected_objects]

    def get_positions(self, reference=None):
        reference = reference or self.reference_frame
        self._positions.clear()
        for obj_class, obj_frame in self._objects:
            if obj_frame:
                try:
                    trans, _ = self.listener.lookupTransform(reference, obj_frame, rospy.Time(0))
                    self._positions[obj_frame] = [trans, obj_class]
                except Exception as e:
                    rospy.loginfo("[Objects] vish!")
                    print(e)
                    self._positions[obj_frame] = [[0.0, 0.0, 0.0], obj_class]

    def _add_object_position(self, x, y, z, obj_string):
        aux = ObjectPosition(x=x, y=y, z=z, rx=0.0, ry=0.0, rz=math.atan2(y, x))
        self._coordinates.append(aux)
        self._taken_object.append(obj_string)

    def _get_closest_object(self, upper_limit, lower_limit):
        dist = float("inf")
        for obj_id, (trans, _) in self._positions.items():
            x, y, z = trans
            if lower_limit < trans[2] < upper_limit:
                value = math.sqrt(x**2 + y**2 + z**2)
                if value < dist:
                    dist = value
                    self._obj = obj_id
        return self._obj

    def handler(self, request):
        condition = request.condition.lower()
        self.get_positions(request.reference)
        self._coordinates = []
        self._taken_object = []

        if condition == 'closest':
            upper_limit = request.upper_limit or float("inf")
            lower_limit = request.lower_limit or -float("inf")
            closest_obj = self._get_closest_object(upper_limit, lower_limit)
            if closest_obj and closest_obj in self._positions:
                self._add_object_position(*self._positions[closest_obj][0], closest_obj.strip('/').split('/')[1])
                succeeded = True
            else:
                self._add_object_position(0.0, 0.0, 0.0, '')
                succeeded = False

        elif condition == "all":
            succeeded = False
            for obj_id, (x, y, z) in self._positions.items():
                obj_string = obj_id.strip('/').split('/')[1]
                self._add_object_position(x, y, z, obj_string)
                succeeded = True

        rospy.loginfo('Found the coordinates!' if succeeded else "I'm a shame. Sorry!")
        return self._coordinates, self._taken_object

    def specific_handler(self, request):
        self._coordinates = ObjectPosition()
        obj = request.type
        detected_obj = None

        self.get_positions()
        for key, value in self._positions.items():
            if value[1] == obj[:-1]:
                detected_obj = value
                break

        if detected_obj:
            self._coordinates.x, self._coordinates.y, self._coordinates.z = detected_obj[0]
            self._coordinates.rx = 0.0
            self._coordinates.ry = 0.0
            self._coordinates.rz = math.atan2(self._coordinates.y, self._coordinates.x)
            succeeded = True
        else:
            self._coordinates.x = self._coordinates.y = self._coordinates.z = 0.0
            self._coordinates.rx = self._coordinates.ry = self._coordinates.rz = 0.0
            succeeded = False

        rospy.loginfo('Found the coordinates!' if succeeded else "I'm a shame. Sorry!")
        return self._coordinates

if __name__ == '__main__':
    rospy.init_node('objects', log_level=rospy.ERROR)
    Objects()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
