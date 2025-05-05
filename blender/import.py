bl_info = {
    "name": "SLRBS Scene Import/Export",
    "author": "Your Name",
    "version": (1, 0),
    "blender": (4, 3, 0),
    "location": "File > Import/Export > SLRBS Scene (.json)",
    "description": "Import and export scenes in SLRBS JSON format",
    "warning": "",
    "doc_url": "",
    "category": "Import-Export",
}

import bpy
import os
import json
import mathutils
from bpy_extras.io_utils import ExportHelper, ImportHelper
from bpy.props import StringProperty, BoolProperty, FloatProperty
from bpy.types import Operator

# -------------------------------------------------------------
# Conversion functions (Z-up <-> Y-up coordinate swap)
# -------------------------------------------------------------

def convert_vector3_to_json(vec):
    return {"x": vec.x, "y": vec.z, "z": -vec.y}


def convert_json_to_vector3(d):
    # SLRBS Y-up to Blender Z-up: Blender = (x, -z, y)
    return mathutils.Vector((d['x'], -d['z'], d['y']))


def convert_quaternion_to_json(quat):
    rot = mathutils.Quaternion((0.7071068, -0.7071068, 0, 0))
    converted = rot @ quat @ rot.conjugated()
    return {"w": converted.w, "x": converted.x, "y": converted.y, "z": converted.z}


def convert_json_to_quaternion(q):
    rot = mathutils.Quaternion((0.7071068, -0.7071068, 0, 0))
    quat = mathutils.Quaternion((q['w'], q['x'], q['y'], q['z']))
    # inverse coordinate conversion
    restored = rot.conjugated() @ quat @ rot
    return restored


def convert_color_to_json(color):
    return {"r": color[0], "g": color[1], "b": color[2]}

# -------------------------------------------------------------
# Geometry mapping: Blender -> JSON
# (create_geometry_json already exists in export)
# -------------------------------------------------------------
def create_geometry_json(obj):
    # ... existing export logic ...
    # (Copy-paste from your exporter)
    pass

# -------------------------------------------------------------
# Geometry mapping: JSON -> Blender
# -------------------------------------------------------------
def create_geometry_from_json(body):
    geom = body['geometry']
    typ = geom['type']
    obj = None
    if typ == 'box':
        dims = geom['dimensions']
        bpy.ops.mesh.primitive_cube_add(size=1)
        obj = bpy.context.active_object
        obj.scale = (dims['x'], dims['y'], dims['z'])
    elif typ == 'sphere':
        r = geom['radius']
        bpy.ops.mesh.primitive_uv_sphere_add(radius=r)
        obj = bpy.context.active_object
    elif typ == 'cylinder':
        r = geom['radius']
        h = geom['height']
        bpy.ops.mesh.primitive_cylinder_add(radius=r, depth=h)
        obj = bpy.context.active_object
    elif typ == 'plane':
        # finite plane to visualize
        bpy.ops.mesh.primitive_plane_add(size=10)
        obj = bpy.context.active_object
        # orient plane normal
        n = convert_json_to_vector3(geom['normal'])
        q = mathutils.Vector((0,0,1)).rotation_difference(n)
        obj.rotation_mode = 'QUATERNION'
        obj.rotation_quaternion = q
    else:
        # default cube
        bpy.ops.mesh.primitive_cube_add(size=1)
        obj = bpy.context.active_object
    return obj

# -------------------------------------------------------------
# Export Operator (unchanged)
# -------------------------------------------------------------
class ExportSLRBSScene(Operator, ExportHelper):
    bl_idname = "export_scene.slrbs"
    bl_label = "Export SLRBS Scene"
    bl_options = {'PRESET', 'UNDO'}
    filename_ext = ".json"
    filter_glob: StringProperty(default="*.json", options={'HIDDEN'}, maxlen=255)
    export_selected: BoolProperty(name="Export Selected Only", default=False)
    scene_name: StringProperty(name="Scene Name", default="BlenderScene")
    scene_description: StringProperty(name="Scene Description", default="Scene exported from Blender")
    gravity_magnitude: FloatProperty(name="Gravity Magnitude", default=9.81, min=0.0, precision=2)

    def execute(self, context):
        # ... your existing export code ...
        return {'FINISHED'}

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False
        layout.prop(self, "scene_name")
        layout.prop(self, "scene_description")
        layout.prop(self, "export_selected")
        layout.prop(self, "gravity_magnitude")

# -------------------------------------------------------------
# Import Operator (new)
# -------------------------------------------------------------
class ImportSLRBSScene(Operator, ImportHelper):
    bl_idname = "import_scene.slrbs"
    bl_label = "Import SLRBS Scene"
    bl_options = {'PRESET', 'UNDO'}

    filename_ext = ".json"
    filter_glob: StringProperty(default="*.json", options={'HIDDEN'}, maxlen=255)
    clear_scene: BoolProperty(name="Clear Scene", default=True, description="Delete existing objects before import")

    def execute(self, context):
        # load JSON
        with open(self.filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)

        # clear scene
        if self.clear_scene:
            bpy.ops.object.select_all(action='SELECT')
            bpy.ops.object.delete()

        # import bodies
        bodies = {}
        for b in data.get('bodies', []):
            obj = create_geometry_from_json(b)
            # set name
            obj.name = f"Body_{b['id']}"
            # set transform
            obj.location = convert_json_to_vector3(b['position'])
            q = convert_json_to_quaternion(b['orientation'])
            obj.rotation_mode = 'QUATERNION'
            obj.rotation_quaternion = q
            # visual color
            col = b.get('visual', {}).get('color', {})
            if obj.data and hasattr(obj.data, 'materials'):
                mat = bpy.data.materials.new(name=obj.name + "_mat")
                mat.diffuse_color = (col.get('r',0.8), col.get('g',0.8), col.get('b',0.8), 1.0)
                obj.data.materials.append(mat)
            bodies[b['id']] = obj

        # joints not yet implemented
        self.report({'INFO'}, f"Imported {len(bodies)} bodies")
        return {'FINISHED'}

# -------------------------------------------------------------
# Menu functions and registration
# -------------------------------------------------------------
def menu_func_export(self, context):
    self.layout.operator(ExportSLRBSScene.bl_idname, text="SLRBS Scene (.json)")

def menu_func_import(self, context):
    self.layout.operator(ImportSLRBSScene.bl_idname, text="SLRBS Scene (.json)")


def register():
    bpy.utils.register_class(ExportSLRBSScene)
    bpy.utils.register_class(ImportSLRBSScene)
    bpy.types.TOPBAR_MT_file_export.append(menu_func_export)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)


def unregister():
    bpy.utils.unregister_class(ExportSLRBSScene)
    bpy.utils.unregister_class(ImportSLRBSScene)
    bpy.types.TOPBAR_MT_file_export.remove(menu_func_export)
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)

if __name__ == "__main__":
    register()
