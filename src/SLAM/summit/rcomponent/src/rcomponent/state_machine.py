#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2022, Robotnik Automation SLL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotnik Automation SSL nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

class StateMachine:
    'Class to represent a state machine'

    def __init__(self, id, available_states):
        # id to identify the component
        self._id = id
        # array of string
        self._available_states = available_states
        self._current_state = ''
        self._desired_state = ''
        self.__invalid_state = ''

    def add_state(self, state):

        pass

    def get_current_state(self):
        return self._current_state

    def get_desired_state(self):
        return self._desired_state

    def is_in_desired_state(self):
        return self._current_state != self.__invalid_state and self._current_state == self._desired_state

    def switch_to_state(self, state):
        if self.check_state_exists(state) == False:
            rospy.logerr('%s:switch_to_state: the state %s is not valid', self._id, state)    
            return False
        
        if state == self._current_state:
            return True
        
        rospy.loginfo('%s:switch_to_state: switching from %s to %s', self._id, self._current_state, state)
        self._current_state = state

        return True
    
    def check_state_exists(self, state):

        return state != self.__invalid_state and (state in self._available_states)

    def switch_to_desired_state(self):

        return self.switch_to_state(self._desired_state)

    def set_desired_state(self, state):

        if self.check_state_exists(state) == False:
            rospy.logerr('%s:set_desired_state: the state %s is not valid', self._id, state)    
        
        self._desired_state = state
        return True
