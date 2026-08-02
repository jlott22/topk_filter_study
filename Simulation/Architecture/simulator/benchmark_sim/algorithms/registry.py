from __future__ import annotations

import importlib
from typing import Type

from .base import AllocatorBase


def load_allocator_class(spec: str) -> Type[AllocatorBase]:
    """Load an allocator class from 'module.path:ClassName'."""
    if ":" not in spec:
        raise ValueError("Algorithm spec must be in the form module.path:ClassName")
    module_name, class_name = spec.split(":", 1)
    module = importlib.import_module(module_name)
    cls = getattr(module, class_name)
    if not issubclass(cls, AllocatorBase):
        raise TypeError(f"{spec} is not an AllocatorBase subclass")
    return cls
