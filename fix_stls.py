import trimesh
import glob
import os

# Find all STL files in your meshes folder
stl_files = glob.glob(os.path.join("meshes", "*.stl"))

print("--- 🔍 STL MESH DIAGNOSTICS ---")
for file_path in stl_files:
    try:
        mesh = trimesh.load(file_path)
        face_count = len(mesh.faces)
        
        status = "✅ PASS" if 0 < face_count <= 200000 else "❌ FAIL"
        
        print(f"{status} | File: {file_path}")
        print(f"         Faces: {face_count:,} (Limit: 200,000)")
        
        if face_count > 200000:
            print("         -> ACTION: You must simplify this mesh in your CAD software!")
        elif face_count == 0:
            print("         -> ACTION: This file is completely empty/corrupted.")
            
    except Exception as e:
        print(f"❌ ERROR reading {file_path}: {e}")
