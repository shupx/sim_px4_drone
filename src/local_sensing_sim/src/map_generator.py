#!/usr/bin/env python3
"""
Random Map Generator with Cylinders and Torus Obstacles
Generates random 3D point cloud maps with configurable field dimensions
@author: Peixuan Shu
@date: Jan 2026
pip3 install open3d numpy pyyaml
"""

import open3d as o3d
import numpy as np
import argparse
import yaml
import os


class MapGenerator:
    def __init__(self, config):
        """
        Initialize the map generator with configuration
        
        Args:
            config: Dictionary containing configuration parameters
        """
        self.config = config
        self.seed = config.get('seed', 42)
        np.random.seed(self.seed)
        
        # Field dimensions
        self.field_x = config.get('field_x', 20.0)
        self.field_y = config.get('field_y', 20.0)
        self.field_z = config.get('field_z', 5.0)
        
        # Obstacle parameters
        self.num_cylinders = config.get('num_cylinders', 10)
        self.num_torus = config.get('num_torus', 5)
        
        # Cylinder parameters
        self.cyl_radius_range = config.get('cyl_radius_range', [0.2, 0.5])
        self.cyl_height_range = config.get('cyl_height_range', [1.0, 3.0])
        self.cyl_points = config.get('cyl_points', 3000)
        
        # Torus parameters
        self.torus_major_radius_range = config.get('torus_major_radius_range', [0.8, 1.5])
        self.torus_minor_radius_range = config.get('torus_minor_radius_range', [0.15, 0.3])
        self.torus_points = config.get('torus_points', 3000)
        
        # Output
        self.output_file = config.get('output_file', 'random_map.pcd')
        
        # Optional: Add boundary walls
        self.add_boundaries = config.get('add_boundaries', False)
        self.boundary_points = config.get('boundary_points', 10000)
        
        # Optional: Add ground plane
        self.add_ground = config.get('add_ground', True)
        self.ground_points = config.get('ground_points', 5000)
        
        self.scene = o3d.geometry.PointCloud()
    
    def generate_cylinder(self):
        """Generate a random cylinder obstacle"""
        radius = np.random.uniform(*self.cyl_radius_range)
        height = np.random.uniform(*self.cyl_height_range)
        
        # Create cylinder
        cyl = o3d.geometry.TriangleMesh.create_cylinder(radius, height)
        
        # Random position within field
        x = np.random.uniform(-self.field_x/2, self.field_x/2)
        y = np.random.uniform(-self.field_y/2, self.field_y/2)
        z = np.random.uniform(height/2, self.field_z - height/2)  # Ensure cylinder fits in field
        
        cyl.translate([x, y, z])
        
        # Sample points from the mesh
        pcd_cyl = cyl.sample_points_uniformly(self.cyl_points)
        
        return pcd_cyl
    
    def generate_torus(self):
        """Generate a random torus (ring) obstacle"""
        major_radius = np.random.uniform(*self.torus_major_radius_range)
        minor_radius = np.random.uniform(*self.torus_minor_radius_range)
        
        # Create torus
        torus = o3d.geometry.TriangleMesh.create_torus(major_radius, minor_radius)
        
        # Random position within field
        x = np.random.uniform(-self.field_x/2 + major_radius, self.field_x/2 - major_radius)
        y = np.random.uniform(-self.field_y/2 + major_radius, self.field_y/2 - major_radius)
        z = np.random.uniform(major_radius, self.field_z - major_radius)
        
        # Random rotation
        R = torus.get_rotation_matrix_from_xyz((
            np.random.uniform(0, 2*np.pi),
            np.random.uniform(0, 2*np.pi),
            np.random.uniform(0, 2*np.pi)
        ))
        torus.rotate(R, center=(0, 0, 0))
        torus.translate([x, y, z])
        
        # Sample points from the mesh
        pcd_torus = torus.sample_points_uniformly(self.torus_points)
        
        return pcd_torus
    
    def generate_ground(self):
        """Generate ground plane"""
        ground = o3d.geometry.TriangleMesh.create_box(
            width=self.field_x,
            height=self.field_y,
            depth=0.1
        )
        ground.translate([-self.field_x/2, -self.field_y/2, -0.1])
        pcd_ground = ground.sample_points_uniformly(self.ground_points)
        return pcd_ground
    
    def generate_boundaries(self):
        """Generate boundary walls around the field"""
        boundaries = o3d.geometry.PointCloud()
        
        # Create 4 walls
        wall_thickness = 0.1
        
        # Wall 1: x = -field_x/2
        wall1 = o3d.geometry.TriangleMesh.create_box(
            width=wall_thickness,
            height=self.field_y,
            depth=self.field_z
        )
        wall1.translate([-self.field_x/2, -self.field_y/2, 0])
        boundaries += wall1.sample_points_uniformly(self.boundary_points // 4)
        
        # Wall 2: x = field_x/2
        wall2 = o3d.geometry.TriangleMesh.create_box(
            width=wall_thickness,
            height=self.field_y,
            depth=self.field_z
        )
        wall2.translate([self.field_x/2 - wall_thickness, -self.field_y/2, 0])
        boundaries += wall2.sample_points_uniformly(self.boundary_points // 4)
        
        # Wall 3: y = -field_y/2
        wall3 = o3d.geometry.TriangleMesh.create_box(
            width=self.field_x,
            height=wall_thickness,
            depth=self.field_z
        )
        wall3.translate([-self.field_x/2, -self.field_y/2, 0])
        boundaries += wall3.sample_points_uniformly(self.boundary_points // 4)
        
        # Wall 4: y = field_y/2
        wall4 = o3d.geometry.TriangleMesh.create_box(
            width=self.field_x,
            height=wall_thickness,
            depth=self.field_z
        )
        wall4.translate([-self.field_x/2, self.field_y/2 - wall_thickness, 0])
        boundaries += wall4.sample_points_uniformly(self.boundary_points // 4)
        
        return boundaries
    
    def generate_map(self):
        """Generate the complete map with all obstacles"""
        print(f"Generating map with seed: {self.seed}")
        print(f"Field dimensions: {self.field_x} x {self.field_y} x {self.field_z}")
        
        # Generate cylinders
        print(f"Generating {self.num_cylinders} cylinders...")
        for i in range(self.num_cylinders):
            pcd_cyl = self.generate_cylinder()
            self.scene += pcd_cyl
        
        # Generate torus obstacles
        print(f"Generating {self.num_torus} torus obstacles...")
        for i in range(self.num_torus):
            pcd_torus = self.generate_torus()
            self.scene += pcd_torus
        
        # Add ground if enabled
        if self.add_ground:
            print("Adding ground plane...")
            pcd_ground = self.generate_ground()
            self.scene += pcd_ground
        
        # Add boundaries if enabled
        if self.add_boundaries:
            print("Adding boundary walls...")
            boundaries = self.generate_boundaries()
            self.scene += boundaries
        
        print(f"Total points in scene: {len(self.scene.points)}")
        
        return self.scene
    
    def save_map(self):
        """Save the generated map to a PCD file"""
        output_path = self.output_file
        
        # Create directory if it doesn't exist
        output_dir = os.path.dirname(output_path)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)
        
        # Save point cloud
        success = o3d.io.write_point_cloud(output_path, self.scene)
        
        if success:
            print(f"Map saved successfully to: {output_path}")
        else:
            print(f"Failed to save map to: {output_path}")
        
        return success
    
    def visualize(self):
        """Visualize the generated map"""
        print("Visualizing map...")
        o3d.visualization.draw_geometries([self.scene],
                                          window_name="Generated Map",
                                          width=1024,
                                          height=768)


def load_config(config_file):
    """Load configuration from YAML file"""
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    return config


def main():
    parser = argparse.ArgumentParser(description='Generate random 3D maps with obstacles')
    parser.add_argument('--config', type=str, help='Path to configuration YAML file')
    parser.add_argument('--seed', type=int, help='Random seed')
    parser.add_argument('--output', type=str, help='Output PCD file path')
    parser.add_argument('--visualize', action='store_true', help='Visualize the generated map')
    parser.add_argument('--field-x', type=float, help='Field width (x dimension)')
    parser.add_argument('--field-y', type=float, help='Field length (y dimension)')
    parser.add_argument('--field-z', type=float, help='Field height (z dimension)')
    parser.add_argument('--num-cylinders', type=int, help='Number of cylinders')
    parser.add_argument('--num-torus', type=int, help='Number of torus obstacles')
    
    args = parser.parse_args()
    
    # Load configuration
    if args.config and os.path.exists(args.config):
        config = load_config(args.config)
    else:
        # Default configuration
        config = {
            'seed': 42,
            'field_x': 20.0,
            'field_y': 20.0,
            'field_z': 5.0,
            'num_cylinders': 10,
            'num_torus': 5,
            'cyl_radius_range': [0.2, 0.5],
            'cyl_height_range': [1.0, 3.0],
            'cyl_points': 3000,
            'torus_major_radius_range': [0.8, 1.5],
            'torus_minor_radius_range': [0.15, 0.3],
            'torus_points': 3000,
            'add_boundaries': False,
            'boundary_points': 10000,
            'add_ground': True,
            'ground_points': 5000,
            'output_file': 'random_map.pcd'
        }
    
    # Override with command line arguments
    if args.seed is not None:
        config['seed'] = args.seed
    if args.output is not None:
        config['output_file'] = args.output
    if args.field_x is not None:
        config['field_x'] = args.field_x
    if args.field_y is not None:
        config['field_y'] = args.field_y
    if args.field_z is not None:
        config['field_z'] = args.field_z
    if args.num_cylinders is not None:
        config['num_cylinders'] = args.num_cylinders
    if args.num_torus is not None:
        config['num_torus'] = args.num_torus
    
    # Create map generator
    generator = MapGenerator(config)
    
    # Generate map
    generator.generate_map()
    
    # Save map
    generator.save_map()
    
    # Visualize if requested
    if args.visualize:
        generator.visualize()


if __name__ == '__main__':
    main()
