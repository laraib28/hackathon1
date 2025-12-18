"""
Base Skill class for the agent skill system.
All skills should inherit from this class.
"""
from abc import ABC, abstractmethod
from typing import Any, Dict, Optional
from dataclasses import dataclass


@dataclass
class SkillResult:
    """Result of a skill execution"""
    success: bool
    data: Any
    error: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None


class BaseSkill(ABC):
    """Abstract base class for all agent skills"""
    
    def __init__(self, name: str, description: str):
        self.name = name
        self.description = description
    
    @abstractmethod
    async def execute(self, *args, **kwargs) -> SkillResult:
        """Execute the skill with given parameters"""
        pass
    
    def validate_input(self, *args, **kwargs) -> bool:
        """Validate input parameters before execution"""
        return True