import imgui
import imgui.integrations.glfw
import glfw
from OpenGL.GL import glClearColor, glClear, GL_COLOR_BUFFER_BIT
from networktables import NetworkTables
from PIL import Image

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
        glfw.set_window_size_callback(self.window, self.resize_callback)

        imgui.create_context()
        imgui.set_current_context(imgui.get_current_context())

        self.impl = imgui.integrations.glfw.GlfwRenderer(self.window)

        self.speed = 0.0
        self.battery = 0.0
        self.quick_reef_one = "UNKNOWN"  # Default value for Quick Reef One
        self.quick_reef_two = "UNKNOWN"  # Default value for Quick Reef Two
        self.quick_reef_three = "UNKNOWN"  # Default value for Quick Reef Three
        self.selected_reef = "UNKNOWN"

        self.image_path = "2025 REEFSCAPE Blue.png"
        try:
            self.image = Image.open(self.image_path)
            self.image_texture = self.load_texture_from_image(self.image)
        except Exception as e:
            print(f"Failed to load image: {e}")
            self.image_texture = None

    def resize_callback(self, window, width, height):
        imgui.get_io().display_size = (width, height)

    def load_texture_from_image(self, image):
        from OpenGL.GL import glGenTextures, glBindTexture, glTexImage2D, GL_TEXTURE_2D, GL_RGBA, GL_UNSIGNED_BYTE, glTexParameteri, GL_TEXTURE_MIN_FILTER, GL_TEXTURE_MAG_FILTER, GL_LINEAR

        image = image.transpose(Image.FLIP_TOP_BOTTOM)
        image_data = image.convert("RGBA").tobytes()
        width, height = image.size

        texture_id = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, texture_id)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)

        return texture_id

    def update_telemetry(self):
        self.speed = self.table.getNumber("Speed", 0.0)
        self.battery = self.table.getNumber("Battery", 0.0)
        self.quick_reef_one = self.table.getString("QuickReefOne", "UNKNOWN")
        self.quick_reef_two = self.table.getString("QuickReefTwo", "UNKNOWN")
        self.quick_reef_three = self.table.getString("QuickReefThree", "UNKNOWN")
        self.selected_reef = self.table.getString("SelectedReef", "UKNOWN")
        

    def draw_image_with_labels(self):
        if self.image_texture:
            imgui.begin("Image with REEF Labels")

            window_width, window_height = glfw.get_window_size(self.window)
            img_width, img_height = 400, 400
            imgui.image(self.image_texture, img_width, img_height)

            labels = ["REEF 1", "REEF 2", "REEF 3", "REEF 4", "REEF 5", "REEF 6"]
            positions = [(190, 100), (300, 175), (300, 275), (190, 325), (90, 275), (90, 175)]

            selected_reef_one = int(self.quick_reef_one) if self.quick_reef_one.isdigit() else -1
            selected_reef_two = int(self.quick_reef_two) if self.quick_reef_two.isdigit() else -1
            selected_reef_three = int(self.quick_reef_three) if self.quick_reef_three.isdigit() else -1

            for i, (x, y) in enumerate(positions):
                imgui.set_cursor_pos((x, y))

                # Determine the color for the current label
                if i + 1 == selected_reef_one or i + 1 == selected_reef_two or i + 1 == selected_reef_three:
                    color = (0.0, 1.0, 0.0, 1.0)  # Green
                else:
                    color = (1.0, 0.0, 0.0, 1.0)  # Red

                imgui.text_colored(labels[i], *color)

            imgui.end()

    def run(self):
        while not glfw.window_should_close(self.window):
            glfw.poll_events()
            self.impl.process_inputs()

            glfw.make_context_current(self.window)
            glClearColor(0.0, 0.0, 0.0, 1.0)
            glClear(GL_COLOR_BUFFER_BIT)

            imgui.new_frame()

            imgui.begin("Driver Dashboard")

            imgui.text(f"Speed: {self.speed:.1f}")
            imgui.text(f"Battery: {self.battery:.1f}")

            quick_reef_one = self.quick_reef_one
            quick_reef_two = self.quick_reef_two
            quick_reef_three = self.quick_reef_three

            imgui.text(f"Quick REEF One: {quick_reef_one}")
            imgui.text(f"Quick REEF Two: {quick_reef_two}")
            imgui.text(f"Quick REEF Three: {quick_reef_three}")

            changed, new_speed = imgui.slider_float("Speed", self.speed, 0.0, 100.0)
            if changed:
                self.table.putNumber("Speed", new_speed)

            if imgui.button("Reset"):
                self.table.putNumber("Speed", 0.0)

            imgui.end()

            self.draw_image_with_labels()

            imgui.render()
            self.impl.render(imgui.get_draw_data())

            glfw.swap_buffers(self.window)
            self.update_telemetry()

        self.impl.shutdown()
        glfw.terminate()

if __name__ == "__main__":
    team_number = 0
    dashboard = DriverDashboard(team_number=team_number)
    dashboard.run()
