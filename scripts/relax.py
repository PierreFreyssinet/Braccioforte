#!/usr/bin/env python

from niryo_one_python_api.niryo_one_api import *
from safe_move import init

init()
niryo = NiryoOne()
niryo.activate_learning_mode(True)

