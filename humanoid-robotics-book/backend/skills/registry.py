"""
Skill Registry for managing and accessing agent skills.
"""
from typing import Dict, Type, Optional, List
from .base import BaseSkill
import importlib
import os


class SkillRegistry:
    """Global registry to manage agent skills"""
    
    def __init__(self):
        self._skills: Dict[str, Type[BaseSkill]] = {}
        self._instances: Dict[str, BaseSkill] = {}
    
    def register(self, skill_class: Type[BaseSkill]) -> None:
        """Register a skill class in the registry"""
        skill_name = skill_class.__name__.replace('Skill', '').lower()
        self._skills[skill_name] = skill_class
        
        # Register with the original class name as well for backward compatibility
        self._skills[skill_class.__name__.lower()] = skill_class
    
    def register_instance(self, instance: BaseSkill) -> None:
        """Register a skill instance directly"""
        skill_name = instance.name.lower().replace(' ', '_')
        self._instances[skill_name] = instance
        self._instances[instance.__class__.__name__.lower()] = instance
    
    def get_skill_class(self, name: str) -> Optional[Type[BaseSkill]]:
        """Get a skill class by name"""
        return self._skills.get(name.lower())
    
    def get_skill_instance(self, name: str) -> Optional[BaseSkill]:
        """Get a skill instance by name (creates new instance if not exists)"""
        # Check if instance already exists
        if name.lower() in self._instances:
            return self._instances[name.lower()]
        
        # Get the class and create instance
        skill_class = self.get_skill_class(name)
        if skill_class:
            instance = skill_class()
            self._instances[name.lower()] = instance
            return instance
        
        return None
    
    def list_skills(self) -> List[str]:
        """List all registered skill names"""
        return list(set(self._skills.keys()) | set(self._instances.keys()))
    
    def load_all_skills(self, skills_dir: str = "skills/core"):
        """Dynamically load all skills from a directory"""
        # Import all skill modules to register them
        import pkgutil
        import skills.core as core_skills_module
        
        for _, module_name, _ in pkgutil.iter_modules(core_skills_module.__path__):
            module = importlib.import_module(f"skills.core.{module_name}")
            for attr_name in dir(module):
                attr = getattr(module, attr_name)
                if (isinstance(attr, type) and 
                    issubclass(attr, BaseSkill) and 
                    attr != BaseSkill):
                    self.register(attr)


# Global registry instance
skill_registry = SkillRegistry()