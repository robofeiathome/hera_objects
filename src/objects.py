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
        rospy.Subscriber('/detector_2d_node/boxes_coordinates', DicBoxes, self.get_detected_objects)
        rospy.Service('objects', FindObject, self.handle_find_object)
        rospy.Service('specific_object', FindSpecificObject, self.handle_find_specific_object)
        self.listener = tf.TransformListener()
        self.reference_frame = '/manip_base_link'
        rospy.loginfo("[Objects] Ready to provide object coordinates.")

    def get_detected_objects(self, data):
        self._objects = [(obj.type.data, obj.tf_id.data) for obj in data.detected_objects if obj.tf_id.data]

    def update_positions(self, reference_frame):
        self._positions.clear()
        for obj_class, obj_frame in self._objects:
            try:
                position, _ = self.listener.lookupTransform(reference_frame, obj_frame, rospy.Time(0))
                self._positions[obj_frame] = (position, obj_class)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.loginfo("[Objects] Error getting position for {}: {}".format(obj_frame, e))

    def handle_find_object(self, request):
        reference = request.reference if request.reference else self.reference_frame
        self.update_positions(reference)
        return self.process_request(request.condition, request.upper_limit, request.lower_limit, request.exclude or [])

    def process_request(self, condition, upper_limit, lower_limit, exclude):
        coordinates, taken_objects = [], []
        if condition.lower() == 'all':
            for obj_id, (position, obj_class) in self._positions.items():
                if self.is_within_limits(position[2], lower_limit, upper_limit):
                    if obj_class not in exclude:
                        coordinates.append(self.create_object_position(position))
                        taken_objects.append(obj_class)
        else:  # Treat 'closest' or any other conditions as 'closest'
            closest_obj, min_distance = None, float('inf')
            for obj_id, (position, obj_class) in self._positions.items():
                if self.is_within_limits(position[2], lower_limit, upper_limit) and obj_class not in exclude:
                    distance = math.sqrt(sum(coord ** 2 for coord in position))
                    if distance < min_distance:
                        closest_obj, min_distance = obj_id, distance
            if closest_obj:
                position, obj_class = self._positions[closest_obj]
                coordinates.append(self.create_object_position(position))
                taken_objects.append(obj_class)
        return coordinates, taken_objects

    def handle_find_specific_object(self, request):
        self.update_positions(self.reference_frame)
        return self.find_specific(request.type, request.condition)

    def find_specific(self, obj_type, condition):
        matches = [(pos, cls) for pos, cls in self._positions.values() if cls == obj_type]
        if condition == 'right':
            chosen = max(matches, key=lambda x: x[0][1]) if matches else None
        elif condition == 'left':
            chosen = min(matches, key=lambda x: x[0][1]) if matches else None
        else:
            chosen = matches[0] if matches else None
        return [self.create_object_position(chosen[0])] if chosen else [self.empty_object_position()]

    def create_object_position(self, position):
        x, y, z = position
        return ObjectPosition(x=x, y=y, z=z, rx=0.0, ry=0.0, rz=math.atan2(y, x))

    def is_within_limits(self, value, lower, upper):
        if not lower and not upper:
            return True
        lower = float('-inf') if lower is None else lower
        upper = float('inf') if upper is None else upper
        return lower < value < upper

    def empty_object_position(self):
        return ObjectPosition(x=0.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=0.0)


if __name__ == '__main__':
    rospy.init_node('objects', log_level=rospy.ERROR)
    obj_system = Objects()
    rospy.spin()
