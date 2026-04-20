import struct
import numpy as np
import matplotlib.pyplot as plt
import os
import sys

def read_eigen_matrix(f):
    rows = struct.unpack('l', f.read(8))[0]
    cols = struct.unpack('l', f.read(8))[0]
    data = f.read(rows * cols * 4) # float32
    return np.frombuffer(data, dtype=np.float32).reshape(rows, cols)

def main():
    if len(sys.argv) > 1:
        filepath = sys.argv[1]
    else:
        # Default path
        filepath = "../gng_online.bin"
        
    if not os.path.exists(filepath):
        print(f"Error: File not found: {filepath}")
        return

    print(f"Loading GNG data from {filepath}...")
    
    with open(filepath, 'rb') as f:
        version = struct.unpack('I', f.read(4))[0]
        print(f"GNG Version: {version}")
        
        node_count = struct.unpack('i', f.read(4))[0]
        print(f"Node Count: {node_count}")
        
        nodes = {} # id -> {angle, coord}
        
        for _ in range(node_count):
            node_id = struct.unpack('i', f.read(4))[0]
            error_angle = struct.unpack('f', f.read(4))[0]
            error_coord = struct.unpack('f', f.read(4))[0]
            
            weight_angle = read_eigen_matrix(f)
            weight_coord = read_eigen_matrix(f)
            
            # Status fields
            level = struct.unpack('i', f.read(4))[0]
            is_surface = struct.unpack('?', f.read(1))[0]
            is_active_surface = struct.unpack('?', f.read(1))[0]
            valid = struct.unpack('?', f.read(1))[0]
            active = struct.unpack('?', f.read(1))[0]
            is_boundary = struct.unpack('?', f.read(1))[0]
            
            ee_direction = read_eigen_matrix(f)
            
            # Manipulability Info
            manipulability = struct.unpack('f', f.read(4))[0]
            min_singular_value = struct.unpack('f', f.read(4))[0]
            joint_limit_score = struct.unpack('f', f.read(4))[0]
            combined_score = struct.unpack('f', f.read(4))[0]
            manip_valid = struct.unpack('?', f.read(1))[0]
            dynamic_manipulability = struct.unpack('f', f.read(4))[0]
            
            jp_size = struct.unpack('i', f.read(4))[0]
            for _ in range(jp_size):
                read_eigen_matrix(f)
                
            nodes[node_id] = {
                'angle': weight_angle.flatten(),
                'coord': weight_coord.flatten()
            }

        # Angle Edges
        edge_count = struct.unpack('i', f.read(4))[0]
        print(f"Angle Edges: {edge_count}")
        
        angle_edge_lengths = []
        for _ in range(edge_count):
            n1 = struct.unpack('i', f.read(4))[0]
            n2 = struct.unpack('i', f.read(4))[0]
            age = struct.unpack('i', f.read(4))[0]
            active = struct.unpack('?', f.read(1))[0]
            
            if active and n1 in nodes and n2 in nodes:
                dist = np.linalg.norm(nodes[n1]['angle'] - nodes[n2]['angle'])
                angle_edge_lengths.append(dist)

        # Coord Edges (Optional read, just ensuring file pointer is correct)
        # We can also plot these if needed
        try:
             coord_edge_count = struct.unpack('i', f.read(4))[0]
             print(f"Coord Edges: {coord_edge_count}")
             coord_edge_lengths = []
             for _ in range(coord_edge_count):
                n1 = struct.unpack('i', f.read(4))[0]
                n2 = struct.unpack('i', f.read(4))[0]
                age = struct.unpack('i', f.read(4))[0]
                active = struct.unpack('?', f.read(1))[0]
                
                if active and n1 in nodes and n2 in nodes:
                    dist = np.linalg.norm(nodes[n1]['coord'] - nodes[n2]['coord'])
                    coord_edge_lengths.append(dist)
        except:
            print("End of file reached or error reading coord edges.")

    # Convert to numpy arrays
    angle_dists = np.array(angle_edge_lengths)
    coord_dists = np.array(coord_edge_lengths) if 'coord_edge_lengths' in locals() else np.array([])

    if len(angle_dists) > 0:
        print(f"Angle Edges Stats (Rad): Mean={np.mean(angle_dists):.4f}, Std={np.std(angle_dists):.4f}, Max={np.max(angle_dists):.4f}")
    
    if len(coord_dists) > 0:
        print(f"Coord Edges Stats (m): Mean={np.mean(coord_dists):.4f}, Std={np.std(coord_dists):.4f}, Max={np.max(coord_dists):.4f}")

    # Plotting
    plt.figure(figsize=(10, 4))
    
    # Check if we have data
    if len(angle_dists) > 0:
        plt.subplot(1, 2, 1)
        plt.hist(angle_dists, bins=50, color='skyblue', edgecolor='black', alpha=0.7)
        plt.xlim(0.4, 2.5) # Based on user request to limit range
        plt.title(r"Joint Space (Rad)" + "\n" + r"$\mu$: " + f"{np.mean(angle_dists):.4f}" + r", $\sigma$: " + f"{np.std(angle_dists):.4f}", fontsize=11)
        plt.xlabel("Edge Length (rad)")
        plt.ylabel("Number of Edges")
        plt.grid(True, linestyle='--', alpha=0.5)

    if len(coord_dists) > 0:
        plt.subplot(1, 2, 2)
        plt.hist(coord_dists, bins=50, color='salmon', edgecolor='black', alpha=0.7)
        plt.title(r"Task Space (m)" + "\n" + r"$\mu$: " + f"{np.mean(coord_dists):.4f}" + r", $\sigma$: " + f"{np.std(coord_dists):.4f}", fontsize=11)
        plt.xlabel("Edge Length (m)")
        plt.ylabel("Number of Edges")
        plt.grid(True, linestyle='--', alpha=0.5)

    plt.tight_layout()
    output_file = "tex/smc2026/smc_uraki/figure/edge_distribution.png"
    plt.savefig(output_file, dpi=300)
    print(f"Plot saved to {output_file}")
    # plt.show() # Uncomment if running locally with display

if __name__ == "__main__":
    main()
