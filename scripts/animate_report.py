import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from bs4 import BeautifulSoup
import numpy as np
import os
import sys

# --- CONFIGURATION ---
# We'll calculate CONTROL_ROOM_POS dynamically based on grid size
CONTROL_ROOM_OFFSET = 4 

def parse_multi_agent_report(file_path):
    """Parses the HTML report into a list of grid states."""
    with open(file_path, 'r') as f:
        soup = BeautifulSoup(f, 'html.parser')
    
    timesteps = []
    for header in soup.find_all('h2'):
        table = header.find_next('table')
        if not table: continue
        
        grid = []
        agents = {}
        rows = table.find_all('tr')
        for r_idx, row in enumerate(rows):
            grid_row = []
            cols = row.find_all('td')
            for c_idx, col in enumerate(cols):
                text = col.text.strip()
                class_name = col.get('class', [])
                
                # Mapping: 0:Empty, 1:Wall, 2:Fire, 3:Smoke, 4:Exit, 5:Entry
                val = 0
                if 'wall' in class_name: val = 1
                elif 'fire' in class_name: val = 2
                elif 'smoke' in class_name: val = 3
                elif 'exit' in class_name: val = 4
                elif 'entry' in class_name: val = 5
                
                if text.startswith('A'):
                    agents[text] = (c_idx, r_idx)
                
                grid_row.append(val)
            grid.append(grid_row)
        timesteps.append({'grid': np.array(grid), 'agents': agents})
    return timesteps

def animate_simulation(data, output_path):
    if not data: return
    
    rows, cols = data[0]['grid'].shape
    # Control room is centered above the grid
    control_room_pos = (cols // 2, -CONTROL_ROOM_OFFSET)
    
    fig, ax = plt.subplots(figsize=(10, 11))
    # Add extra space at the top for the Control Room
    plt.subplots_adjust(top=0.8)

    def update(frame_idx):
        ax.clear()
        step = data[frame_idx]
        grid = step['grid']
        agents = step['agents']
        
        # 1. Draw the Base Layer
        # Colors: LightGray, DarkGray(Wall), Red(Fire), Gray(Smoke), Green(Exit), Blue(Entry)
        cmap = plt.cm.colors.ListedColormap(['#ecf0f1', '#2c3e50', '#e74c3c', '#95a5a6', '#27ae60', '#2980b9'])
        ax.imshow(grid, cmap=cmap, origin='upper', extent=[-0.5, cols-0.5, rows-0.5, -0.5])
        
        # 2. Draw the Control Room (Outside Grid)
        ax.plot(control_room_pos[0], control_room_pos[1], marker='H', markersize=30, color='#2c3e50')
        ax.text(control_room_pos[0], control_room_pos[1]-1.5, "CONTROL ROOM\n(Central Server)", 
                ha='center', va='center', fontweight='bold', color='#2c3e50', fontsize=10)

        # 3. Draw Grid Lines for clarity
        ax.set_xticks(np.arange(-0.5, cols, 1), minor=True)
        ax.set_yticks(np.arange(-0.5, rows, 1), minor=True)
        ax.grid(which='minor', color='w', linestyle='-', linewidth=1)

        # 4. Draw Agents as Small Humanoid Icons
        for name, pos in agents.items():
            # Draw Head & Body (Humanoid shape using circle + triangle)
            ax.plot(pos[0], pos[1]-0.1, 'o', color='white', markersize=8, zorder=6) # Head
            ax.plot(pos[0], pos[1]+0.2, '^', color='white', markersize=12, zorder=6) # Body
            ax.text(pos[0], pos[1]+0.7, name, color='black', fontsize=7, ha='center', fontweight='bold')
            
            # Communication Line to Control Room
            ax.plot([pos[0], control_room_pos[0]], [pos[1], control_room_pos[1]], 
                    color='#3498db', linestyle='--', alpha=0.3, linewidth=0.8, zorder=1)

        # 5. Highlight Key Areas
        for r in range(rows):
            for c in range(cols):
                if grid[r, c] == 4: # EXIT
                    ax.text(c, r, "EXIT", color='white', fontsize=7, ha='center', va='center', fontweight='bold')
                elif grid[r, c] == 5: # ENTRY
                    ax.text(c, r, "IN", color='white', fontsize=7, ha='center', va='center', fontweight='bold')
                elif grid[r, c] == 2: # FIRE (Add a small glow)
                    ax.plot(c, r, marker='*', color='#f1c40f', markersize=5, alpha=0.6)

        ax.set_title(f"Real-Time CPS Navigation\nStep: {frame_idx}", pad=60, fontsize=14)
        ax.set_xlim(-1, cols)
        ax.set_ylim(rows, -CONTROL_ROOM_OFFSET - 2)
        ax.axis('off')

    ani = animation.FuncAnimation(fig, update, frames=len(data), interval=300)
    ani.save(output_path, writer='pillow')
    plt.close()

if __name__ == "__main__":
    html_path = sys.argv[1] if len(sys.argv) > 1 else "multi_agent_report.html"
    if os.path.exists(html_path):
        data = parse_multi_agent_report(html_path)
        output_gif = os.path.join(os.path.dirname(html_path), "evacuation_replay.gif")
        print(f"[Python] Creating visual replay: {output_gif}")
        animate_simulation(data, output_gif)
    else:
        print(f"[Error] File not found: {html_path}")