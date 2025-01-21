import imgui
import imgui.integrations.glfw
import glfw
from OpenGL.GL import glClearColor, glClear, GL_COLOR_BUFFER_BIT
from networktables import NetworkTables
import math

class DriverDashboard:

    def __init__(self, team_number):
        # Initialize NetworkTables for communication
        NetworkTables.initialize(server=f"127.0.0.1" if team_number == 0 else f"roborio-{team_number}-frc.local")
        self.table = NetworkTables.getTable("SmartDashboard")

        # Initialize GLFW and ImGui
        if not glfw.init():
            raise Exception("GLFW could not be initialized")

        self.window = glfw.create_window(800, 600, "FRC Driver Dashboard", None, None)
        if not self.window:
            glfw.terminate()
            raise Exception("GLFW window could not be created")

        glfw.make_context_current(self.window)

        # Create and set the ImGui context
        imgui.create_context()
        imgui.set_current_context(imgui.get_current_context())

        self.impl = imgui.integrations.glfw.GlfwRenderer(self.window)

        self.speed = 0.0
        self.battery = 0.0
        self.quick_reef_one = ""

    def update_telemetry(self):
        # Retrieve telemetry values from NetworkTables
        self.speed = self.table.getNumber("Speed", 0.0)
        self.battery = self.table.getNumber("Battery", 0.0)
        self.quick_reef_one = self.table.getString("QuickReefOne", "")

    def get_quick_reef_one(self):
        return self.quick_reef_one

    def draw_hexagon_with_labels(self):
        # Define hexagon parameters
        center_x, center_y = 400, 300  # Center of the hexagon in the window
        radius = 100  # Radius of the hexagon
        angles = [90, 30, -30, -90, -150, -210]  # Angles for the hexagon vertices (clockwise)

        imgui.begin("Hexagon with Labels")

        # Draw the hexagon and labels
        for i, angle in enumerate(angles):
            radians = math.radians(angle)
            x = int(center_x + radius * math.cos(radians))
            y = int(center_y - radius * math.sin(radians))

            # Draw a rectangle at each hexagon vertex to represent a REEF
            rect_width, rect_height = 60, 30
            imgui.set_cursor_pos((x - rect_width // 2, y - rect_height // 2))
            if imgui.button(f"REEF {i + 1}", width=rect_width, height=rect_height):
                print(f"Button REEF {i + 1} clicked")

        imgui.end()

    def run(self):
        while not glfw.window_should_close(self.window):
            glfw.poll_events()
            self.impl.process_inputs()

            # Clear the screen
            glfw.make_context_current(self.window)
            glClearColor(0.0, 0.0, 0.0, 1.0)
            glClear(GL_COLOR_BUFFER_BIT)

            imgui.new_frame()

            imgui.begin("Driver Dashboard")

            # Telemetry display
            imgui.text(f"Speed: {self.speed:.1f}")
            imgui.text(f"Battery: {self.battery:.1f}")

            # Display Quick REEF One
            quick_reef = self.get_quick_reef_one()
            imgui.text(f"Quick REEF One: {quick_reef}")

            # Speed control slider
            changed, new_speed = imgui.slider_float("Speed", self.speed, 0.0, 100.0)
            if changed:
                self.table.putNumber("Speed", new_speed)

            # Reset button
            if imgui.button("Reset"):
                self.table.putNumber("Speed", 0.0)

            imgui.end()

            # Draw the hexagon with REEF labels
            self.draw_hexagon_with_labels()

            imgui.render()
            self.impl.render(imgui.get_draw_data())

            glfw.swap_buffers(self.window)
            self.update_telemetry()

        self.impl.shutdown()
        glfw.terminate()

if __name__ == "__main__":
    team_number = 0  # Set to 0 for simulator mode, replace with actual number for robot connection
    dashboard = DriverDashboard(team_number=team_number)
    dashboard.run()
