## handle_mocap_data extracts head data from the mocap message through NatNet
  #
  #
  # @param self The object pointer
  # @param data The mocap data ROS message
  def handle_mocap_data(self, data):
      if len(data.positions) < 4:
          return
      marker0 = data.positions[0]
      marker1 = data.positions[1]
      marker2 = data.positions[2]
      marker3 = data.positions[3]
      ## In our 3D print, one of the markers is deviated, this is fixed here:
      # marker3.z -= 0.005

      ## Finding the position of the virual centre of mass point:
      body = Point()
      body.x = np.mean([marker0.x,marker1.x,marker2.x,marker3.x])
      body.y = np.mean([marker0.y,marker1.y,marker2.y,marker3.y])
      body.z = np.mean([marker0.z,marker1.z,marker2.z,marker3.z])

      ## Finding the vectors defining the rotated coordinate system of the lens
      v30 = np.array(((marker3.x-marker0.x),(marker3.y-marker0.y),(marker3.z-marker0.z)))
      v01 = np.array(((marker0.x-marker1.x),(marker0.y-marker1.y),(marker0.z-marker1.z)))
      # v10 = np.array(((marker1.x-marker0.x),(marker1.y-marker0.y),(marker1.z-marker0.z)))
      d30 = vector_norm(v30)
      d01 = vector_norm(v01)
      # d10 = vector_norm(v10)
      ## Normalising the vectors, and creating the normal vector
      # ux = v30/d30
      # uz = -v10/d10
      uz = v01/d01
      # vy = np.cross(uz,ux)Final position and orientaion of the lens - the numerical values are
      # to make up for the translation between the centre of mass of the mocap
      # object and the lens, which is slightly in front and below that point.
      # Note that these are still in Mocap coordinates
      vy = np.cross(uz,v30/d30)
      dvy = vector_norm(vy)
      uy = vy/dvy
      ux = np.cross(uy,uz)
      ## Rotation matrix can be obtained as follows:
      rot = np.array((ux,uy,uz)).transpose()
      offset = np.array((controller.enhance_constants.LENS_OFFSET[0],
                         controller.enhance_constants.LENS_OFFSET[1],
                         controller.enhance_constants.LENS_OFFSET[2]))

      offset_eyes = np.array((controller.enhance_constants.EYES_OFFSET[0],
                              controller.enhance_constants.EYES_OFFSET[1],
                              controller.enhance_constants.EYES_OFFSET[2]))
      rotated_offset = np.dot(rot,offset)
      rotated_offset_eyes = np.dot(rot,offset_eyes)

      ## For debugging purposes, we find values in Euler to publish on a topic
      # the below calculations are to achieve this
      rot_for_topic = np.array((np.append(ux,0),np.append(uy,0),np.append(uz,0),np.array((0,0,0,1)))).transpose()
      quat = quaternion_from_matrix(rot_for_topic)
      quat_adapted = [quat[0],-quat[1],-quat[2],-quat[3]]
      euler = list(euler_from_quaternion(quat_adapted))

      ## Final position and orientaion of the lens - the numerical values are
      # to make up for the translation between the centre of mass of the mocap
      # object and the lens, which is slightly in front and below that point.
      # Note that these are still in Mocap coordinates
      self.lens_pose.head_trans_x = body.x + rotated_offset[0]
      self.lens_pose.head_trans_y = body.y + rotated_offset[1]
      self.lens_pose.head_trans_z = body.z + rotated_offset[2]
      # self.lens_pose.head_trans_x = body.x
      # self.lens_pose.head_trans_y = body.y
      # self.lens_pose.head_trans_z = body.z
      self.lens_pose.head_rotation_x = -euler[0]*180.0/np.pi
      self.lens_pose.head_rotation_y = euler[1]*180.0/np.pi
      self.lens_pose.head_rotation_z = euler[2]*180.0/np.pi
      self.mocap_pub.publish(self.lens_pose)

      eyes_position = np.array([body.x + rotated_offset_eyes[0],
                               body.y + rotated_offset_eyes[1],
                               body.z + rotated_offset_eyes[2]])

      eyes_position = np.append(eyes_position,1)
      self.eyes_position = self.enh_controller.array_to_point(self.gaze_calc_handler.get_gaze_point_robot(eyes_position))
      self.eye_pos_pub.publish(self.eyes_position)

      ## Lens position translation from Mocap origin:
      trans = np.array([self.lens_pose.head_trans_x,
                    self.lens_pose.head_trans_y,
                  self.lens_pose.head_trans_z])
      ## Full transformation matrix to be used in gaze calculations:
      self.gaze_calc_handler.head_transform_matrix = np.array([[rot[0][0], rot[0][1], rot[0][2], trans[0]],
                                                               [rot[1][0], rot[1][1], rot[1][2], trans[1]],
                                                               [rot[2][0], rot[2][1], rot[2][2], trans[2]],
                                                               [0, 0, 0, 1]])
      transform = [rot[0][0],rot[0][1],rot[0][2],trans[0],
                   rot[1][0],rot[1][1],rot[1][2],trans[1],
                   rot[2][0],rot[2][1],rot[2][2],trans[2]]

      self.head_data.appendleft(transform)
