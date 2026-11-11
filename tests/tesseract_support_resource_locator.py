from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.common import GeneralResourceLocator


def TesseractSupportResourceLocator():
    """A GeneralResourceLocator that resolves package://tesseract/support/... URLs.

    Searches tesseract's support data (the `[support]` extra, or the conda env's share/ for an
    editable install) as an explicit path, plus the resource env vars it always reads.
    """
    return GeneralResourceLocator(paths=[get_tesseract_resource_path()])
