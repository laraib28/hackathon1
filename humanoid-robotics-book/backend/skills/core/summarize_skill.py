"""
Summarize skill implementation.
"""
from typing import Any, Dict, Optional
from .base import BaseSkill, SkillResult
import logging

logger = logging.getLogger(__name__)


class SummarizeSkill(BaseSkill):
    """Skill for summarizing content"""
    
    def __init__(self, name: str = "summarize", description: str = "Summarize content with specified length"):
        super().__init__(name, description)
        self.max_input_length = 10000  # characters
        self.default_ratio = 0.3  # 30% of original length
    
    async def execute(self, content: str, length: str = "medium", **kwargs) -> SkillResult:
        """
        Execute content summarization
        
        Args:
            content: Content to summarize
            length: Desired length ("short", "medium", "long", or custom ratio)
            
        Returns:
            SkillResult containing summary
        """
        try:
            # Validate input
            if not content or len(content.strip()) == 0:
                return SkillResult(
                    success=False,
                    data=None,
                    error="Content cannot be empty"
                )
            
            if len(content) > self.max_input_length:
                # Truncate or handle large content
                content = content[:self.max_input_length]
            
            # Determine summary ratio based on length preference
            ratio = self._get_ratio(length)
            
            # Perform summarization
            summary = await self._summarize_content(content, ratio)
            
            return SkillResult(
                success=True,
                data=summary,
                metadata={
                    "original_length": len(content),
                    "summary_length": len(summary),
                    "ratio": ratio,
                    "length_preference": length
                }
            )
                
        except Exception as e:
            logger.error(f"Error in SummarizeSkill: {str(e)}")
            return SkillResult(
                success=False,
                data=None,
                error=str(e)
            )
    
    def _get_ratio(self, length: str) -> float:
        """Convert length preference to summary ratio"""
        ratios = {
            "short": 0.15,
            "medium": 0.3,
            "long": 0.5
        }
        return ratios.get(length.lower(), self.default_ratio)
    
    async def _summarize_content(self, content: str, ratio: float) -> str:
        """
        Internal method to perform actual summarization.
        This could use various techniques like extractive or abstractive summarization.
        """
        # This is a simple extractive summarization for demonstration
        # In a real implementation, you might use transformers, gpt, or other NLP models
        sentences = content.split('. ')
        
        if len(sentences) <= 1:
            # If only one sentence, return as is (but limit length if needed)
            return sentences[0][:int(len(sentences[0]) * ratio)] if ratio < 1.0 else sentences[0]
        
        # Calculate how many sentences to include based on ratio
        num_sentences = max(1, int(len(sentences) * ratio))
        
        # For demo purposes, take the first few sentences
        # In a real implementation, you would use a proper summarization algorithm
        selected_sentences = sentences[:num_sentences]
        summary = '. '.join(selected_sentences)
        
        # Ensure it ends properly
        if summary and not summary.endswith('.'):
            summary += '.'
        
        return summary
    
    def validate_input(self, content: str, **kwargs) -> bool:
        """Validate summarization parameters"""
        if not content or len(content.strip()) == 0:
            return False
        if len(content) > self.max_input_length * 10:  # 10x tolerance
            return False
        return True


# Register the skill automatically
from .registry import skill_registry
skill_registry.register(SummarizeSkill)