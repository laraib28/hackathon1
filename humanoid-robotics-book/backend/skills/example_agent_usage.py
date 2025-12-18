"""
Example implementation showing how the main agent would call a skill
"""
from skills.registry import skill_registry
from skills.base import SkillResult
from typing import Any, Dict


class MainAgent:
    """Example main agent that can utilize skills"""
    
    def __init__(self):
        self.skill_registry = skill_registry
    
    async def call_skill(self, skill_name: str, **kwargs) -> SkillResult:
        """
        Generic method to call any skill by name
        
        Args:
            skill_name: Name of the skill to call
            **kwargs: Parameters for the skill
            
        Returns:
            SkillResult from the executed skill
        """
        # Get the skill from registry
        skill = self.skill_registry.get_skill_instance(skill_name)
        
        if not skill:
            return SkillResult(
                success=False,
                data=None,
                error=f"Skill '{skill_name}' not found"
            )
        
        # Execute the skill
        result = await skill.execute(**kwargs)
        return result
    
    async def search_knowledge_base(self, query: str, top_k: int = 5) -> SkillResult:
        """Example specific method that uses the search skill"""
        return await self.call_skill("search", query=query, top_k=top_k)
    
    async def summarize_content(self, content: str, length: str = "medium") -> SkillResult:
        """Example specific method that uses the summarize skill"""
        return await self.call_skill("summarize", content=content, length=length)
    
    async def translate_content(self, content: str, target_lang: str = "ur") -> SkillResult:
        """Example specific method that uses the translate skill"""
        return await self.call_skill("translate", content=content, target_lang=target_lang)
    
    async def process_user_request(self, request: str) -> Dict[str, Any]:
        """
        Example method that demonstrates a multi-skill workflow
        """
        # Step 1: Search for relevant information
        search_result = await self.search_knowledge_base(request, top_k=3)
        
        if not search_result.success:
            return {
                "success": False,
                "error": search_result.error,
                "response": "I couldn't find relevant information for your request."
            }
        
        # Step 2: Summarize the search results
        search_data = search_result.data
        content_to_summarize = "\n".join([item["content"] for item in search_data]) if search_data else ""
        
        if content_to_summarize:
            summary_result = await self.summarize_content(content_to_summarize, "medium")
            
            if summary_result.success:
                return {
                    "success": True,
                    "response": summary_result.data,
                    "metadata": {
                        "search_info": search_result.metadata,
                        "summary_info": summary_result.metadata
                    }
                }
        
        # Fallback response if no content to summarize
        return {
            "success": True,
            "response": f"I searched for '{request}' but couldn't find detailed information to summarize.",
            "metadata": {"search_info": search_result.metadata}
        }


# Example usage:
async def example_usage():
    agent = MainAgent()
    
    # Example 1: Direct skill call
    search_result = await agent.call_skill("search", query="humanoid robotics", top_k=3)
    print(f"Search result: {search_result.data}")
    
    # Example 2: Using specific method
    content = "Artificial intelligence is a wonderful field that combines computer science and data to enable machines to learn and make decisions."
    summary_result = await agent.summarize_content(content, length="short")
    print(f"Summary: {summary_result.data}")
    
    # Example 3: Multi-step workflow
    user_request = "Explain how humanoid robots use AI"
    response = await agent.process_user_request(user_request)
    print(f"Agent response: {response['response']}")


if __name__ == "__main__":
    import asyncio
    asyncio.run(example_usage())