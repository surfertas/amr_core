#!/usr/bin/env python
import tensorflow as tf
import os


class Controller(object):

    def __init__(self, model_path, model_name):
        self._model_path = model_path
        self._model_name = model_name
        self._graph = self._init_graph()
        self._sess = tf.Session(graph=self.graph)

    def _init_graph(self):
        """ Initialize graph from saved model. """
         
        path_to_model = os.path.join(self._model_path, self._model_name)
        graph = tf.Graph()
        with graph.as_default():
            graph_def = tf.GraphDef()
            with tf.gfile.GFile(path_to_model, 'rb') as fid:
                serialized_graph = fid.read()
                graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(graph_def, name='')

        rospy.loginfo('Controller graph initialized...')
        return graph


    def run_inference(self, image):
        """ Runs inference. """ 
        input_tensor = self._graph.get_tensor_by_name('input:0')
        output_tensor = self._graph.get_tensor_by_name('output:0')
    
        return self._sess.run(output_tensor, feed_dict={input_tensor: image})
