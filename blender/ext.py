bl_info = {
    "name": "SLRBS Scene Exporter",
    "author": "Your Name",
    "version": (1, 0),
    "blender": (4, 3, 0),
    "location": "File > Export > SLRBS Scene (.json)",
    "description": "Export scenes to SLRBS JSON format",
    "warning": "",
    "doc_url": "",
    "category": "Import-Export",
}

import bpy
import os
import json
import math
import mathutils
from bpy_extras.io_utils import ExportHelper
from bpy.props import StringProperty, BoolProperty, EnumProperty, FloatProperty
from bpy.types import Operator

# Conversion functions
def convert_vector3_to_json(vec):
    """Convert a Vector3 to JSON format with Y-up to Z-up conversion"""
    # Convert from Blender's Z-up to SLRBS Y-up coordinate system
    return {"x": vec.x, "y": vec.z, "z": -vec.y}

def convert_quaternion_to_json(quat):
    """Convert a Quaternion to JSON format with coordinate system conversion"""
    # Create a rotation to convert from Z-up to Y-up
    rot = mathutils.Quaternion((0.7071068, -0.7071068, 0, 0))
    # Apply rotation to convert coordinate systems
    converted_quat = rot @ quat @ rot.conjugated()
    return {"w": converted_quat.w, "x": converted_quat.x, "y": converted_quat.y, "z": converted_quat.z}

def convert_color_to_json(color):
    """Convert a Color to JSON format"""
    return {"r": color[0], "g": color[1], "b": color[2]}

def create_geometry_json(obj):
    """Create geometry JSON based on object type"""
    if obj.type == 'MESH':
        # Check dimensions to determine shape
        dims = obj.dimensions
        
        # If all dimensions are similar, treat as sphere
        if abs(dims.x - dims.y) < 0.01 and abs(dims.x - dims.z) < 0.01:
            radius = dims.x / 2.0
            return {"type": "sphere", "radius": radius}
        
        # If one dimension is significantly smaller than others, treat as a cylinder
        elif (dims.x < 0.1 * dims.y and dims.x < 0.1 * dims.z) or \
             (dims.y < 0.1 * dims.x and dims.y < 0.1 * dims.z) or \
             (dims.z < 0.1 * dims.x and dims.z < 0.1 * dims.y):
            
            # Determine which axis is height
            if dims.z > dims.x and dims.z > dims.y:
                height = dims.z
                radius = max(dims.x, dims.y) / 2.0
            elif dims.y > dims.x and dims.y > dims.z:
                height = dims.y
                radius = max(dims.x, dims.z) / 2.0
            else:
                height = dims.x
                radius = max(dims.y, dims.z) / 2.0
                
            return {"type": "cylinder", "height": height, "radius": radius}
        
        # Default to box
        else:
            # Convert dimensions to SLRBS coordinate system (y-up)
            converted_dims = mathutils.Vector((dims.x, dims.z, dims.y))
            return {
                "type": "box", 
                "dimensions": {
                    "x": converted_dims.x,
                    "y": converted_dims.y,
                    "z": converted_dims.z
                }
            }
            
    elif obj.type == 'EMPTY' and obj.empty_display_type == 'PLAIN_AXES':
        # Treat empty objects with plain axes as planes
        # Convert normal from Blender's Z-up to SLRBS Y-up coordinate system
        blender_normal = obj.matrix_world.to_3x3() @ mathutils.Vector((0, 0, 1))
        slrbs_normal = mathutils.Vector((blender_normal.x, blender_normal.z, -blender_normal.y))
        
        return {
            "type": "plane",
            "point": convert_vector3_to_json(obj.location),
            "normal": {"x": slrbs_normal.x, "y": slrbs_normal.y, "z": slrbs_normal.z}
        }
        
    # Default to box with dimensions of 1
    return {"type": "box", "dimensions": {"x": 1.0, "y": 1.0, "z": 1.0}}

def convert_gravity(gravity_vec):
    """Convert gravity vector from Blender to SLRBS coordinate system"""
    # In Blender, default gravity is (0, 0, -9.81) for Z-up
    # In SLRBS, gravity should be (0, -9.81, 0) for Y-up
    return {"x": gravity_vec.x, "y": -gravity_vec.z, "z": gravity_vec.y}

class ExportSLRBSScene(Operator, ExportHelper):
    """Export the scene to SLRBS JSON format"""
    bl_idname = "export_scene.slrbs"
    bl_label = "Export SLRBS Scene"
    bl_options = {'PRESET', 'UNDO'}

    # ExportHelper mixin class uses this
    filename_ext = ".json"

    filter_glob: StringProperty(
        default="*.json",
        options={'HIDDEN'},
        maxlen=255,
    )

    # Export options
    export_selected: BoolProperty(
        name="Export Selected Only",
        description="Export only selected objects",
        default=False,
    )
    
    convert_units: BoolProperty(
        name="Convert Units to Meters",
        description="Convert Blender units to meters",
        default=True,
    )
    
    scene_name: StringProperty(
        name="Scene Name",
        description="Name of the exported scene",
        default="BlenderScene",
    )
    
    scene_description: StringProperty(
        name="Scene Description",
        description="Description of the exported scene",
        default="Scene exported from Blender",
    )
    
    gravity_magnitude: FloatProperty(
        name="Gravity Magnitude",
        description="Magnitude of gravity force",
        default=9.81,
        min=0.0,
        precision=2,
    )

    def execute(self, context):
        # Get objects to export
        if self.export_selected:
            objects = context.selected_objects
        else:
            objects = context.scene.objects
            
        # Filter for mesh objects and empties (for planes)
        valid_objects = [obj for obj in objects if obj.type in ('MESH', 'EMPTY')]
        
        if not valid_objects:
            self.report({'ERROR'}, "No valid objects to export")
            return {'CANCELLED'}
        
        # Create JSON structure
        scene_json = {
            "name": self.scene_name,
            "description": self.scene_description,
            "version": 1.0,
            "bodies": [],
            "joints": [],
            "physics": {
                "gravity": {
                    "x": 0.0,
                    "y": -self.gravity_magnitude,
                    "z": 0.0
                },
                "time_step": 0.01,  # Default value
                "solver_iterations": 10  # Default value
            }
        }
        
        # Process objects
        body_id = 0
        body_map = {}  # Map Blender objects to their IDs
        
        for obj in valid_objects:
            # Create body JSON
            body_json = {
                "id": body_id,
                "mass": obj.rigid_body.mass if hasattr(obj, 'rigid_body') and obj.rigid_body else 1.0,
                "fixed": obj.rigid_body.type == 'PASSIVE' if hasattr(obj, 'rigid_body') and obj.rigid_body else False,
                "position": convert_vector3_to_json(obj.location),
                "orientation": convert_quaternion_to_json(obj.rotation_quaternion if obj.rotation_mode == 'QUATERNION' else obj.rotation_euler.to_quaternion()),
                "linear_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0}
            }
            
            # Add rigid body velocities if available
            if hasattr(obj, 'rigid_body') and obj.rigid_body and hasattr(obj.rigid_body, 'linear_velocity'):
                body_json["linear_velocity"] = convert_vector3_to_json(obj.rigid_body.linear_velocity)
                
            if hasattr(obj, 'rigid_body') and obj.rigid_body and hasattr(obj.rigid_body, 'angular_velocity'):
                body_json["angular_velocity"] = convert_vector3_to_json(obj.rigid_body.angular_velocity)
            
            # Add geometry
            body_json["geometry"] = create_geometry_json(obj)
            
            # Add visual properties
            if obj.type == 'MESH' and obj.active_material:
                color = obj.active_material.diffuse_color[:3]
                transparency = 1.0 - obj.active_material.diffuse_color[3]
            else:
                color = (0.8, 0.8, 0.8)
                transparency = 0.0
                
            body_json["visual"] = {
                "color": convert_color_to_json(color),
                "transparency": transparency,
                "smooth_shade": obj.data.use_auto_smooth if hasattr(obj, 'data') and hasattr(obj.data, 'use_auto_smooth') else True,
                "edge_width": 1.0
            }
            
            # Add to bodies
            scene_json["bodies"].append(body_json)
            body_map[obj] = body_id
            body_id += 1
        
        # Process constraints (for joints)
        joint_id = 0
        
        for obj in bpy.data.objects:
            if obj.constraints:
                for constraint in obj.constraints:
                    # Only process constraints between objects we've exported
                    if constraint.type in ('PIVOT', 'HINGE', 'BALL') and \
                       hasattr(constraint, 'target') and constraint.target in body_map and obj in body_map:
                        
                        body0_id = body_map[obj]
                        body1_id = body_map[constraint.target]
                        
                        # Create joint JSON
                        joint_json = {
                            "id": joint_id,
                            "body0_id": body0_id,
                            "body1_id": body1_id
                        }
                        
                        # Get offset points
                        if hasattr(constraint, 'pivot_x'):
                            joint_json["body0_offset"] = convert_vector3_to_json(constraint.pivot_x)
                        else:
                            joint_json["body0_offset"] = {"x": 0, "y": 0, "z": 0}
                            
                        if hasattr(constraint, 'pivot_y'):
                            joint_json["body1_offset"] = convert_vector3_to_json(constraint.pivot_y)
                        else:
                            joint_json["body1_offset"] = {"x": 0, "y": 0, "z": 0}
                        
                        # Set joint type and specific properties
                        if constraint.type == 'HINGE':
                            joint_json["type"] = "hinge"
                            # Calculate orientations
                            q0 = mathutils.Quaternion((1, 0, 0, 0))  # Default identity quaternion
                            q1 = mathutils.Quaternion((1, 0, 0, 0))
                            
                            joint_json["body0_orientation"] = convert_quaternion_to_json(q0)
                            joint_json["body1_orientation"] = convert_quaternion_to_json(q1)
                            
                        elif constraint.type in ('PIVOT', 'BALL'):
                            joint_json["type"] = "spherical"
                        
                        # Add joint to scene
                        scene_json["joints"].append(joint_json)
                        joint_id += 1
        
        # Write JSON to file
        with open(self.filepath, 'w', encoding='utf-8') as f:
            json.dump(scene_json, f, indent=4)
            
        self.report({'INFO'}, f"Exported {len(scene_json['bodies'])} bodies and {len(scene_json['joints'])} joints to {self.filepath}")
        return {'FINISHED'}

    def draw(self, context):
        layout = self.layout
        
        layout.use_property_split = True
        layout.use_property_decorate = False
        
        layout.prop(self, "scene_name")
        layout.prop(self, "scene_description")
        layout.prop(self, "export_selected")
        layout.prop(self, "convert_units")
        layout.prop(self, "gravity_magnitude")

def menu_func_export(self, context):
    self.layout.operator(ExportSLRBSScene.bl_idname, text="SLRBS Scene (.json)")

def register():
    bpy.utils.register_class(ExportSLRBSScene)
    bpy.types.TOPBAR_MT_file_export.append(menu_func_export)

def unregister():
    bpy.utils.unregister_class(ExportSLRBSScene)
    bpy.types.TOPBAR_MT_file_export.remove(menu_func_export)

if __name__ == "__main__":
    register()