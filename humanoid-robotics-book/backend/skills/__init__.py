"""
Skills package initialization
"""
from .registry import skill_registry

# Import all core skills to ensure they are registered
from .core import search_skill
from .core import summarize_skill
from .core import translate_skill

__all__ = ['skill_registry', 'BaseSkill', 'SkillResult']