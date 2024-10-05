#!/usr/bin/env python3

# Author: Brubru

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
        self._specific = {0: [0.0, 0.0, 0.0]}
        self.obj_tf_id = ""
        self.reference_frame = '/manip_base_link'

        rospy.Subscriber('/detector_2d_node/boxes_coordinates', DicBoxes, self.get_detected_objects)
        rospy.Service('objects', FindObject, self.handler)
        rospy.Service('specific_object', FindSpecificObject, self.specific_handler)

        self.listener = tf.TransformListener()
        rospy.loginfo("[Objects] Dear Operator, I'm ready to give you object coordinates")

    def get_detected_objects(self, array):
        """ Recebe os objetos detectados e atualiza a lista """
        if array.detected_objects:
            self._objects = [(detection.type.data, detection.tf_id.data) for detection in array.detected_objects]

    def get_positions(self, reference=None):
        """ Calcula a posição dos objetos detectados em relação ao frame de referência """
        reference = reference or self.reference_frame
        self._positions.clear()

        for obj_class, obj_frame in self._objects:
            if obj_frame:
                try:
                    trans, _ = self.listener.lookupTransform(reference, obj_frame, rospy.Time(0))
                    self._positions[obj_frame] = [trans, obj_class]
                except Exception as e:
                    rospy.loginfo("[Objects] Error obtaining object position")
                    print(e)
                    self._positions[obj_frame] = [0.0, 0.0, 0.0]
                    self._specific = {0: [0.0, 0.0, 0.0]}

    def handler(self, request):
        """ Manipula as requisições de objetos e retorna as coordenadas de acordo com a condição """
        self.get_positions(request.reference)
        condition = request.condition.lower()
        self._coordinates = []
        self._taken_object = []

        if condition == 'closest':
            return self._handle_closest_condition(request)
        elif condition == 'all':
            return self._handle_all_condition(request)
        else:
            return self._handle_default_condition()

    def _handle_closest_condition(self, request):
        """ Lógica para encontrar o objeto mais próximo """
        dist = float("inf")
        self._obj = None
        for obj_id, position in self._positions.items():
            x, y, z = position[0]
            if self._is_within_limits(obj_id, request):
                value = self._calculate_distance(x, y, z)
                if value < dist:
                    dist = value
                    self._obj = obj_id

        return self._build_response(succeeded=bool(self._obj))

    def _handle_all_condition(self, request):
        """ Lógica para retornar todos os objetos dentro dos limites """
        for obj_id, position in self._positions.items():
            if self._is_within_limits(obj_id, request):
                self._add_object_to_response(obj_id)

        return self._build_response(succeeded=bool(self._coordinates))

    def _handle_default_condition(self):
        """ Lógica para condição padrão (sem filtro) """
        aux = ObjectPosition(x=0.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=0.0)
        self._coordinates.append(aux)
        self._taken_object.append('')
        rospy.loginfo("I'm a shame. Sorry!")
        return self._coordinates, self._taken_object

    def _is_within_limits(self, obj_id, request):
        """ Verifica se o objeto está dentro dos limites superior e inferior """
        trans, _ = self.listener.lookupTransform("map", obj_id, rospy.Time(0))
        return request.lower_limit < trans[2] < request.upper_limit and obj_id.strip('/').split('/')[1][:-1] not in request.exclude

    def _calculate_distance(self, x, y, z):
        """ Calcula a distância de um objeto a partir de suas coordenadas """
        return math.sqrt(x ** 2 + y ** 2 + z ** 2)

    def _add_object_to_response(self, obj_id):
        """ Adiciona objeto detectado à lista de resposta """
        x, y, z = self._positions[obj_id][0]
        obj_string = obj_id.strip('/').split('/')[1]
        aux = ObjectPosition(x=x, y=y, z=z, rx=0.0, ry=0.0, rz=math.atan2(y, x))
        self._coordinates.append(aux)
        self._taken_object.append(obj_string)

    def _build_response(self, succeeded):
        """ Constroi a resposta baseada no status de sucesso """
        if succeeded and self._obj in self._positions:
            self._add_object_to_response(self._obj)
        else:
            self._add_empty_object()
        rospy.loginfo('Found the coordinates!') if succeeded else rospy.loginfo("I'm a shame. Sorry!")
        return self._coordinates, self._taken_object

    def _add_empty_object(self):
        """ Adiciona um objeto vazio (padrão) à resposta """
        aux = ObjectPosition(x=0.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=0.0)
        self._coordinates.append(aux)
        self._taken_object.append('')

    def specific_handler(self, request):
        """ Manipula requisições para objetos específicos """
        obj_type = request.type[:-1]  # Remove o último caractere (ex: número no final)
        condition = request.condition.lower()
        self.get_positions()
        detected_obj = [(position[0], key.strip('/').split('/')[1]) for key, position in self._positions.items() if position[1] == obj_type]

        if condition == '':
            self._specific = detected_obj[0] if detected_obj else {0: [0.0, 0.0, 0.0]}
        elif condition == 'right':
            self._specific = min(detected_obj, key=lambda obj_id: obj_id[0][1], default={0: [0.0, 0.0, 0.0]})
        elif condition == 'left':
            self._specific = max(detected_obj, key=lambda obj_id: obj_id[0][1], default={0: [0.0, 0.0, 0.0]})

        return self._build_specific_response()

    def _build_specific_response(self):
        """ Constroi a resposta para um objeto específico """
        if self._specific:
            x, y, z = self._specific[0]
            self._coordinates = ObjectPosition(x=x, y=y, z=z, rx=0.0, ry=0.0, rz=math.atan2(y, x))
            succeeded = True
        else:
            self._coordinates = ObjectPosition(x=0.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=0.0)
            succeeded = False

        rospy.loginfo('Found the coordinates!') if succeeded else rospy.loginfo("I'm a shame. Sorry!")
        return self._coordinates, self._specific[1] if self._specific else ''

if __name__ == '__main__':
    rospy.init_node('objects', log_level=rospy.ERROR)
    Objects()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
