
"use strict";

let display_draw_pixel = require('./display_draw_pixel.js')
let display_draw_oval = require('./display_draw_oval.js')
let set_string = require('./set_string.js')
let node_get_orientation = require('./node_get_orientation.js')
let gps_decimal_degrees_to_degrees_minutes_seconds = require('./gps_decimal_degrees_to_degrees_minutes_seconds.js')
let field_set_vec3f = require('./field_set_vec3f.js')
let display_image_paste = require('./display_image_paste.js')
let node_set_velocity = require('./node_set_velocity.js')
let field_get_count = require('./field_get_count.js')
let node_add_force_with_offset = require('./node_add_force_with_offset.js')
let node_add_force_or_torque = require('./node_add_force_or_torque.js')
let speaker_is_sound_playing = require('./speaker_is_sound_playing.js')
let field_set_bool = require('./field_set_bool.js')
let field_get_rotation = require('./field_get_rotation.js')
let receiver_get_emitter_direction = require('./receiver_get_emitter_direction.js')
let field_set_string = require('./field_set_string.js')
let motor_set_control_pid = require('./motor_set_control_pid.js')
let get_string = require('./get_string.js')
let field_get_float = require('./field_get_float.js')
let skin_set_bone_position = require('./skin_set_bone_position.js')
let field_set_int32 = require('./field_set_int32.js')
let supervisor_set_label = require('./supervisor_set_label.js')
let mouse_get_state = require('./mouse_get_state.js')
let display_draw_text = require('./display_draw_text.js')
let robot_get_device_list = require('./robot_get_device_list.js')
let supervisor_movie_stop_recording = require('./supervisor_movie_stop_recording.js')
let node_get_velocity = require('./node_get_velocity.js')
let camera_get_zoom_info = require('./camera_get_zoom_info.js')
let display_draw_line = require('./display_draw_line.js')
let get_int = require('./get_int.js')
let robot_set_mode = require('./robot_set_mode.js')
let node_set_visibility = require('./node_set_visibility.js')
let skin_get_bone_position = require('./skin_get_bone_position.js')
let field_set_vec2f = require('./field_set_vec2f.js')
let speaker_play_sound = require('./speaker_play_sound.js')
let display_image_load = require('./display_image_load.js')
let field_import_node_from_string = require('./field_import_node_from_string.js')
let lidar_get_frequency_info = require('./lidar_get_frequency_info.js')
let field_remove = require('./field_remove.js')
let range_finder_get_info = require('./range_finder_get_info.js')
let lidar_get_info = require('./lidar_get_info.js')
let camera_get_info = require('./camera_get_info.js')
let field_get_type_name = require('./field_get_type_name.js')
let display_draw_rectangle = require('./display_draw_rectangle.js')
let node_get_field = require('./node_get_field.js')
let node_get_parent_node = require('./node_get_parent_node.js')
let field_get_bool = require('./field_get_bool.js')
let node_get_contact_point = require('./node_get_contact_point.js')
let get_bool = require('./get_bool.js')
let supervisor_get_from_id = require('./supervisor_get_from_id.js')
let skin_set_bone_orientation = require('./skin_set_bone_orientation.js')
let display_image_save = require('./display_image_save.js')
let camera_get_focus_info = require('./camera_get_focus_info.js')
let field_get_string = require('./field_get_string.js')
let get_uint64 = require('./get_uint64.js')
let set_int = require('./set_int.js')
let node_get_id = require('./node_get_id.js')
let supervisor_virtual_reality_headset_get_orientation = require('./supervisor_virtual_reality_headset_get_orientation.js')
let field_get_node = require('./field_get_node.js')
let lidar_get_layer_range_image = require('./lidar_get_layer_range_image.js')
let field_set_rotation = require('./field_set_rotation.js')
let node_move_viewpoint = require('./node_move_viewpoint.js')
let display_set_font = require('./display_set_font.js')
let set_float_array = require('./set_float_array.js')
let lidar_get_layer_point_cloud = require('./lidar_get_layer_point_cloud.js')
let field_import_node = require('./field_import_node.js')
let node_get_static_balance = require('./node_get_static_balance.js')
let node_get_type = require('./node_get_type.js')
let skin_get_bone_name = require('./skin_get_bone_name.js')
let node_remove = require('./node_remove.js')
let display_image_delete = require('./display_image_delete.js')
let node_get_name = require('./node_get_name.js')
let display_image_new = require('./display_image_new.js')
let field_get_vec3f = require('./field_get_vec3f.js')
let node_get_status = require('./node_get_status.js')
let supervisor_get_from_def = require('./supervisor_get_from_def.js')
let set_bool = require('./set_bool.js')
let field_get_color = require('./field_get_color.js')
let speaker_speak = require('./speaker_speak.js')
let supervisor_movie_start_recording = require('./supervisor_movie_start_recording.js')
let display_draw_polygon = require('./display_draw_polygon.js')
let set_float = require('./set_float.js')
let node_get_center_of_mass = require('./node_get_center_of_mass.js')
let get_float = require('./get_float.js')
let field_remove_node = require('./field_remove_node.js')
let skin_get_bone_orientation = require('./skin_get_bone_orientation.js')
let field_get_int32 = require('./field_get_int32.js')
let display_get_info = require('./display_get_info.js')
let field_get_type = require('./field_get_type.js')
let field_set_float = require('./field_set_float.js')
let robot_wait_for_user_input_event = require('./robot_wait_for_user_input_event.js')
let node_get_position = require('./node_get_position.js')
let field_set_color = require('./field_set_color.js')
let pen_set_ink_color = require('./pen_set_ink_color.js')
let node_get_number_of_contact_points = require('./node_get_number_of_contact_points.js')
let field_get_vec2f = require('./field_get_vec2f.js')
let supervisor_virtual_reality_headset_get_position = require('./supervisor_virtual_reality_headset_get_position.js')
let save_image = require('./save_image.js')
let node_reset_functions = require('./node_reset_functions.js')
let display_image_copy = require('./display_image_copy.js')

module.exports = {
  display_draw_pixel: display_draw_pixel,
  display_draw_oval: display_draw_oval,
  set_string: set_string,
  node_get_orientation: node_get_orientation,
  gps_decimal_degrees_to_degrees_minutes_seconds: gps_decimal_degrees_to_degrees_minutes_seconds,
  field_set_vec3f: field_set_vec3f,
  display_image_paste: display_image_paste,
  node_set_velocity: node_set_velocity,
  field_get_count: field_get_count,
  node_add_force_with_offset: node_add_force_with_offset,
  node_add_force_or_torque: node_add_force_or_torque,
  speaker_is_sound_playing: speaker_is_sound_playing,
  field_set_bool: field_set_bool,
  field_get_rotation: field_get_rotation,
  receiver_get_emitter_direction: receiver_get_emitter_direction,
  field_set_string: field_set_string,
  motor_set_control_pid: motor_set_control_pid,
  get_string: get_string,
  field_get_float: field_get_float,
  skin_set_bone_position: skin_set_bone_position,
  field_set_int32: field_set_int32,
  supervisor_set_label: supervisor_set_label,
  mouse_get_state: mouse_get_state,
  display_draw_text: display_draw_text,
  robot_get_device_list: robot_get_device_list,
  supervisor_movie_stop_recording: supervisor_movie_stop_recording,
  node_get_velocity: node_get_velocity,
  camera_get_zoom_info: camera_get_zoom_info,
  display_draw_line: display_draw_line,
  get_int: get_int,
  robot_set_mode: robot_set_mode,
  node_set_visibility: node_set_visibility,
  skin_get_bone_position: skin_get_bone_position,
  field_set_vec2f: field_set_vec2f,
  speaker_play_sound: speaker_play_sound,
  display_image_load: display_image_load,
  field_import_node_from_string: field_import_node_from_string,
  lidar_get_frequency_info: lidar_get_frequency_info,
  field_remove: field_remove,
  range_finder_get_info: range_finder_get_info,
  lidar_get_info: lidar_get_info,
  camera_get_info: camera_get_info,
  field_get_type_name: field_get_type_name,
  display_draw_rectangle: display_draw_rectangle,
  node_get_field: node_get_field,
  node_get_parent_node: node_get_parent_node,
  field_get_bool: field_get_bool,
  node_get_contact_point: node_get_contact_point,
  get_bool: get_bool,
  supervisor_get_from_id: supervisor_get_from_id,
  skin_set_bone_orientation: skin_set_bone_orientation,
  display_image_save: display_image_save,
  camera_get_focus_info: camera_get_focus_info,
  field_get_string: field_get_string,
  get_uint64: get_uint64,
  set_int: set_int,
  node_get_id: node_get_id,
  supervisor_virtual_reality_headset_get_orientation: supervisor_virtual_reality_headset_get_orientation,
  field_get_node: field_get_node,
  lidar_get_layer_range_image: lidar_get_layer_range_image,
  field_set_rotation: field_set_rotation,
  node_move_viewpoint: node_move_viewpoint,
  display_set_font: display_set_font,
  set_float_array: set_float_array,
  lidar_get_layer_point_cloud: lidar_get_layer_point_cloud,
  field_import_node: field_import_node,
  node_get_static_balance: node_get_static_balance,
  node_get_type: node_get_type,
  skin_get_bone_name: skin_get_bone_name,
  node_remove: node_remove,
  display_image_delete: display_image_delete,
  node_get_name: node_get_name,
  display_image_new: display_image_new,
  field_get_vec3f: field_get_vec3f,
  node_get_status: node_get_status,
  supervisor_get_from_def: supervisor_get_from_def,
  set_bool: set_bool,
  field_get_color: field_get_color,
  speaker_speak: speaker_speak,
  supervisor_movie_start_recording: supervisor_movie_start_recording,
  display_draw_polygon: display_draw_polygon,
  set_float: set_float,
  node_get_center_of_mass: node_get_center_of_mass,
  get_float: get_float,
  field_remove_node: field_remove_node,
  skin_get_bone_orientation: skin_get_bone_orientation,
  field_get_int32: field_get_int32,
  display_get_info: display_get_info,
  field_get_type: field_get_type,
  field_set_float: field_set_float,
  robot_wait_for_user_input_event: robot_wait_for_user_input_event,
  node_get_position: node_get_position,
  field_set_color: field_set_color,
  pen_set_ink_color: pen_set_ink_color,
  node_get_number_of_contact_points: node_get_number_of_contact_points,
  field_get_vec2f: field_get_vec2f,
  supervisor_virtual_reality_headset_get_position: supervisor_virtual_reality_headset_get_position,
  save_image: save_image,
  node_reset_functions: node_reset_functions,
  display_image_copy: display_image_copy,
};
