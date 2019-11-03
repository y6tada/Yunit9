#!/usr/bin/env python
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np

file = "val_trace.txt"
with open(file, "w", encoding = "utf_8") as fileobj:
	tmp = 3.14
	fileobj.write(str(tmp))
