import logging
import argparse

# ANSI color codes
COLORS = {
    'HEADER': '\033[95m',
    'BLUE': '\033[94m',
    'GREEN': '\033[92m',
    'YELLOW': '\033[93m',
    'RED': '\033[91m',
    'ENDC': '\033[0m',
    'BOLD': '\033[1m',
    'UNDERLINE': '\033[4m'
}

parser = argparse.ArgumentParser(description="",
                                 usage="",
                                 epilog="",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('-v', '--vehicle_id', type=str, default='', help='vehicle id')
parser.add_argument('-n', '--node', type=str, default='ros_to_kafka', help='node name')
parsed_known_args, unknown_args = parser.parse_known_args()


class ColoredFormatter(logging.Formatter):
    def __init__(self, fmt=None, datefmt=None):
        super().__init__(fmt, datefmt)
        self.datefmt = datefmt

    def format(self, record):
        if record.levelno == logging.CRITICAL:
            prefix = COLORS['RED'] + COLORS['BOLD']
        elif record.levelno == logging.ERROR:
            prefix = COLORS['RED']
        elif record.levelno == logging.WARNING:
            prefix = COLORS['YELLOW']
        elif record.levelno == logging.INFO:
            prefix = COLORS['GREEN']
        elif record.levelno == logging.DEBUG:
            prefix = COLORS['BLUE']
        else:
            prefix = COLORS['ENDC']

        # Add color and then format the log record
        formatted_message = prefix + super().format(record) + COLORS['ENDC']
        return formatted_message


# Configure the handler with time formatting
time_format = '%Y-%m-%d %H:%M:%S'
log_format = '%(asctime)s - %(levelname)s - %(message)s'

handler = logging.StreamHandler()
handler.setFormatter(ColoredFormatter(fmt=log_format, datefmt=time_format))

logger = logging.getLogger('colored')
logger.addHandler(handler)
logger.setLevel(logging.DEBUG)


def debug(message):
    logger.debug(message)


def info(message):
    logger.info(message)


def warning(message):
    logger.warning(message)


def error(message):
    logger.error(message)


def critical(message):
    logger.critical(message)


'''
logger.debug("This is a debug message")
logger.info("This is an info message")
logger.warning("This is a warning message")
logger.error("This is an error message")
logger.critical("This is a critical message")
'''