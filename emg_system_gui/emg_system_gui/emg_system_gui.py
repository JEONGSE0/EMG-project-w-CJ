import sys
import traceback
from PySide6.QtWidgets import QApplication
from .main_window import MainWindow


def main(args=None):
    try:
        app = QApplication(sys.argv)
        window = MainWindow()
        window.show()

        exit_code = app.exec()

        try:
            window.close()
        except Exception:
            pass

        return exit_code

    except Exception:
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())