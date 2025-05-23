#!/usr/bin/env python3
"""
IFC Construction Site Generator
Creates a simplified construction site IFC file suitable for conversion to Gazebo
"""

import ifcopenshell
import ifcopenshell.api
import numpy as np
from datetime import datetime

def create_construction_site():
    """Create a simplified construction site IFC model"""
    
    # Create a new IFC file
    ifc = ifcopenshell.api.run("project.create_file", version="IFC4")
    
    # Create project context
    project = ifcopenshell.api.run("root.create_entity", ifc, ifc_class="IfcProject", name="Construction Site")
    
    # Create units
    ifcopenshell.api.run("unit.assign_unit", ifc, length={"is_metric": True, "raw": "METERS"})
    
    # Create site
    site = ifcopenshell.api.run("root.create_entity", ifc, ifc_class="IfcSite", name="Construction Site Area")
    ifcopenshell.api.run("aggregate.assign_object", ifc, products=[site], relating_object=project)
    
    # Create building (construction in progress)
    building = ifcopenshell.api.run("root.create_entity", ifc, ifc_class="IfcBuilding", name="Building Under Construction")
    ifcopenshell.api.run("aggregate.assign_object", ifc, products=[building], relating_object=site)
    
    # Create ground floor storey
    storey = ifcopenshell.api.run("root.create_entity", ifc, ifc_class="IfcBuildingStorey", name="Ground Floor")
    ifcopenshell.api.run("aggregate.assign_object", ifc, products=[storey], relating_object=building)
    
    # Create context for geometry
    context = ifcopenshell.api.run("context.add_context", ifc, context_type="Model")
    body = ifcopenshell.api.run("context.add_context", ifc, 
                               context_type="Model", 
                               context_identifier="Body", 
                               target_view="MODEL_VIEW", 
                               parent=context)
    
    # Helper function to create a wall
    def create_wall(name, start_point, end_point, height=3.0, thickness=0.2):
        wall = ifcopenshell.api.run("root.create_entity", ifc, ifc_class="IfcWall", name=name)
        
        # Calculate wall direction and length
        dx = end_point[0] - start_point[0]
        dy = end_point[1] - start_point[1]
        length = np.sqrt(dx**2 + dy**2)
        angle = np.arctan2(dy, dx)
        
        # Create a rectangular profile directly
        profile = ifc.create_entity("IfcRectangleProfileDef",
                                  ProfileType="AREA",
                                  XDim=length,
                                  YDim=thickness)
        
        # Create extrusion
        direction = ifc.create_entity("IfcDirection", DirectionRatios=[0.0, 0.0, 1.0])
        extrusion = ifc.create_entity("IfcExtrudedAreaSolid",
                                    SweptArea=profile,
                                    ExtrudedDirection=direction,
                                    Depth=height)
        
        # Create shape representation
        shape_representation = ifc.create_entity("IfcShapeRepresentation",
                                               ContextOfItems=body,
                                               RepresentationIdentifier="Body",
                                               RepresentationType="SweptSolid",
                                               Items=[extrusion])
        
        # Create product representation
        representation = ifc.create_entity("IfcProductDefinitionShape",
                                         Representations=[shape_representation])
        
        # Assign representation
        wall.Representation = representation
        
        # Create placement
        matrix = np.eye(4)
        matrix[0][3] = start_point[0] + dx/2
        matrix[1][3] = start_point[1] + dy/2
        matrix[2][3] = height/2
        
        # Apply rotation
        cos_a = np.cos(angle)
        sin_a = np.sin(angle)
        matrix[0][0] = cos_a
        matrix[0][1] = -sin_a
        matrix[1][0] = sin_a
        matrix[1][1] = cos_a
        
        ifcopenshell.api.run("geometry.edit_object_placement", ifc, product=wall, matrix=matrix)
        
        # Assign to storey
        ifcopenshell.api.run("spatial.assign_container", ifc, products=[wall], relating_structure=storey)
        
        return wall
    
    # Helper function to create a slab
    def create_slab(name, points, thickness=0.2, elevation=0.0):
        slab = ifcopenshell.api.run("root.create_entity", ifc, ifc_class="IfcSlab", name=name)
        
        # Create a polygonal profile from points
        # First, create the polyline
        polyline_points = []
        for point in points:
            polyline_points.append(ifc.create_entity("IfcCartesianPoint", Coordinates=point))
        
        # Close the polyline by adding the first point at the end
        polyline_points.append(polyline_points[0])
        
        polyline = ifc.create_entity("IfcPolyline", Points=polyline_points)
        
        # Create an arbitrary closed profile
        profile = ifc.create_entity("IfcArbitraryClosedProfileDef",
                                  ProfileType="AREA",
                                  OuterCurve=polyline)
        
        # Create extrusion
        direction = ifc.create_entity("IfcDirection", DirectionRatios=[0.0, 0.0, 1.0])
        extrusion = ifc.create_entity("IfcExtrudedAreaSolid",
                                    SweptArea=profile,
                                    ExtrudedDirection=direction,
                                    Depth=thickness)
        
        # Create shape representation
        shape_representation = ifc.create_entity("IfcShapeRepresentation",
                                               ContextOfItems=body,
                                               RepresentationIdentifier="Body",
                                               RepresentationType="SweptSolid",
                                               Items=[extrusion])
        
        # Create product representation
        representation = ifc.create_entity("IfcProductDefinitionShape",
                                         Representations=[shape_representation])
        
        # Assign representation
        slab.Representation = representation
        
        # Create placement
        matrix = np.eye(4)
        matrix[2][3] = elevation
        
        ifcopenshell.api.run("geometry.edit_object_placement", ifc, product=slab, matrix=matrix)
        
        # Assign to storey
        ifcopenshell.api.run("spatial.assign_container", ifc, products=[slab], relating_structure=storey)
        
        return slab
    
    # Helper function to create a column
    def create_column(name, position, height=3.0, size=0.3):
        column = ifcopenshell.api.run("root.create_entity", ifc, ifc_class="IfcColumn", name=name)
        
        # Create a square profile directly
        profile = ifc.create_entity("IfcRectangleProfileDef",
                                  ProfileType="AREA",
                                  XDim=size,
                                  YDim=size)
        
        # Create extrusion
        direction = ifc.create_entity("IfcDirection", DirectionRatios=[0.0, 0.0, 1.0])
        extrusion = ifc.create_entity("IfcExtrudedAreaSolid",
                                    SweptArea=profile,
                                    ExtrudedDirection=direction,
                                    Depth=height)
        
        # Create shape representation
        shape_representation = ifc.create_entity("IfcShapeRepresentation",
                                               ContextOfItems=body,
                                               RepresentationIdentifier="Body",
                                               RepresentationType="SweptSolid",
                                               Items=[extrusion])
        
        # Create product representation
        representation = ifc.create_entity("IfcProductDefinitionShape",
                                         Representations=[shape_representation])
        
        # Assign representation
        column.Representation = representation
        
        # Create placement
        matrix = np.eye(4)
        matrix[0][3] = position[0]
        matrix[1][3] = position[1]
        matrix[2][3] = height/2
        
        ifcopenshell.api.run("geometry.edit_object_placement", ifc, product=column, matrix=matrix)
        
        # Assign to storey
        ifcopenshell.api.run("spatial.assign_container", ifc, products=[column], relating_structure=storey)
        
        return column
    
    # Create foundation slab
    foundation_points = [
        [0.0, 0.0, 0.0],
        [20.0, 0.0, 0.0],
        [20.0, 15.0, 0.0],
        [0.0, 15.0, 0.0]
    ]
    create_slab("Foundation", foundation_points, thickness=0.5, elevation=-0.5)
    
    # Create partial walls (under construction)
    # Perimeter walls
    create_wall("South Wall", (0, 0), (20, 0), height=2.5)  # Partial height
    create_wall("East Wall", (20, 0), (20, 15), height=3.0)  # Full height
    create_wall("North Wall - Section 1", (20, 15), (10, 15), height=3.0)  # Partial wall
    # Gap in north wall for construction access
    create_wall("West Wall", (0, 15), (0, 0), height=1.5)  # Partial height
    
    # Interior walls (some complete, some partial)
    create_wall("Interior Wall 1", (10, 0), (10, 8), height=3.0)
    create_wall("Interior Wall 2", (10, 8), (20, 8), height=2.0)  # Partial height
    
    # Create columns
    # Grid of columns for structure
    column_positions = [
        (5, 5), (10, 5), (15, 5),
        (5, 10), (10, 10), (15, 10)
    ]
    
    for i, pos in enumerate(column_positions):
        create_column(f"Column {i+1}", pos, height=3.0)
    
    # Create construction elements (temporary structures)
    # Scaffolding representation (simplified as thin columns)
    scaffolding_positions = [
        (0.5, 0.5), (0.5, 14.5),
        (19.5, 0.5), (19.5, 14.5)
    ]
    
    for i, pos in enumerate(scaffolding_positions):
        create_column(f"Scaffolding {i+1}", pos, height=4.0, size=0.1)
    
    # Create a ramp for vehicle access
    ramp_points = [
        [22.0, 5.0, 0.0],
        [25.0, 5.0, 0.0],
        [25.0, 10.0, 0.0],
        [22.0, 10.0, 0.0]
    ]
    create_slab("Access Ramp", ramp_points, thickness=0.3, elevation=0.0)
    
    # Create material storage area (concrete pad)
    storage_points = [
        [5.0, 17.0, 0.0],
        [15.0, 17.0, 0.0],
        [15.0, 22.0, 0.0],
        [5.0, 22.0, 0.0]
    ]
    create_slab("Material Storage Area", storage_points, thickness=0.2, elevation=0.0)
    
    # Add some construction barriers (simplified as low walls)
    create_wall("Barrier 1", (-2, -2), (22, -2), height=1.0, thickness=0.1)
    create_wall("Barrier 2", (22, -2), (22, 17), height=1.0, thickness=0.1)
    create_wall("Barrier 3", (22, 17), (-2, 17), height=1.0, thickness=0.1)
    create_wall("Barrier 4", (-2, 17), (-2, -2), height=1.0, thickness=0.1)
    
    return ifc

def save_construction_site(filename="construction_site.ifc"):
    """Generate and save the construction site IFC file"""
    ifc = create_construction_site()
    ifc.write(filename)
    print(f"Construction site IFC file saved as: {filename}")
    return filename

if __name__ == "__main__":
    # Generate the IFC file
    save_construction_site()
