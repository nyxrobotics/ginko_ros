#!/usr/bin/env python
# -*- coding: utf-8 -*-

import codecs
import string
import re

in_file  = codecs.open('./tmp/sdf/ginko_common_closed.sdf', 'r', 'utf-8')
out_file = codecs.open('./tmp/sdf/ginko_common_closed_spring.sdf', 'w', 'utf-8')

all_text = in_file.read()
out_text = re.sub(r"""
    <joint name='spring(.)_(.+)' type='prismatic'>
      <child>(.+)</child>
      <parent>(.+)</parent>
      <axis>
        <xyz>(.+)</xyz>
        <limit>
          <lower>(.+)</lower>
          <upper>(.+)</upper>
          <effort>(.+)</effort>
          <velocity>(.+)</velocity>
        </limit>
        <dynamics>
          <damping>(.+)</damping>
          <friction>(.+)</friction>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
        </dynamics>
        <use_parent_model_frame>1</use_parent_model_frame>
      </axis>
    </joint>
""",r"""
    <joint name='spring\1_\2' type='prismatic'>
      <child>\3</child>
      <parent>\4</parent>
      <axis>
        <xyz>\5</xyz>
        <limit>
          <lower>\6</lower>
          <upper>\7</upper>
          <effort>\8</effort>
          <velocity>\9</velocity>
        </limit>
        <dynamics>
          <damping>\10</damping>
          <friction>\11</friction>
          <spring_reference>0.0517</spring_reference>
          <spring_stiffness>914.0</spring_stiffness>
        </dynamics>
        <use_parent_model_frame>1</use_parent_model_frame>
      </axis>
    </joint>
""", all_text)
out_file.write(out_text)
in_file.close()
out_file.close()