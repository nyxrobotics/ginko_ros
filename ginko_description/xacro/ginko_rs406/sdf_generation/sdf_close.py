#!/usr/bin/env python
# -*- coding: utf-8 -*-

import codecs
import string
import re

in_file  = codecs.open('./export/sdf/ginko_common.sdf', 'r', 'utf-8')
out_file = codecs.open('./export/sdf/ginko_common_closed.sdf', 'w', 'utf-8')

all_text = in_file.read()
out_text = re.sub(r"""
    <link name='dummy(.)_(.+)'>
      <pose frame=''>(.+)</pose>
      <inertial>
        <pose frame=''>(.+)</pose>
        <mass>(.+)</mass>
        <inertia>
          <ixx>(.+)</ixx>
          <ixy>(.+)</ixy>
          <ixz>(.+)</ixz>
          <iyy>(.+)</iyy>
          <iyz>(.+)</iyz>
          <izz>(.+)</izz>
        </inertia>
      </inertial>
    </link>
    <joint name='(.+)' type='revolute'>
      <child>dummy(.)_(.+)</child>
""",r"""
    <joint name='\12' type='revolute'>
      <pose>\4</pose>
      <child>\14</child>
""", all_text)
out_file.write(out_text)
in_file.close()
out_file.close()