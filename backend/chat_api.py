"""
Chat API Router - Production Ready
RAG-powered chat with OpenAI + Qdrant search
"""

from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import List, Optional
import os
from dotenv import load_dotenv
from openai import OpenAI
from sqlalchemy.orm import Session
from datetime import datetime

# Load environment variables
load_dotenv()

# Import search and database
import search as search_module
from database import get_db, ChatHistory, get_user_by_id, save_content_preference

# Create router
router = APIRouter(prefix="/api")

# Initialize OpenAI
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
if OPENAI_API_KEY:
    openai_client = OpenAI(api_key=OPENAI_API_KEY)
else:
    print("⚠️  WARNING: OPENAI_API_KEY not found in environment variables")
    openai_client = None

# -------------------------------------------------------
# MODELS
# -------------------------------------------------------

class ChatRequest(BaseModel):
    message: str
    target_language: Optional[str] = "en"
    selected_text: Optional[str] = None
    conversation_history: Optional[List[dict]] = []
    user_id: Optional[str] = None

class ChatResponse(BaseModel):
    response: str
    sources: List[dict]
    language: str

class PersonalizeRequest(BaseModel):
    content: str
    user_background: dict
    chapter_id: Optional[str] = None
    user_id: Optional[str] = None

class PersonalizeResponse(BaseModel):
    success: bool
    personalized_content: str

class TranslateRequest(BaseModel):
    content: str
    target_language: str

class TranslateResponse(BaseModel):
    success: bool
    translated_text: str

# -------------------------------------------------------
# ROUTES
# -------------------------------------------------------

@router.post("/chat", response_model=ChatResponse)
async def chat(
    request: ChatRequest,
    db: Session = Depends(get_db)
):
    """
    RAG-powered chat endpoint

    **Features:**
    - Retrieves relevant context from Qdrant
    - Generates response using OpenAI
    - Supports English and Urdu
    - Logs conversation if user_id provided

    **Parameters:**
    - **message**: User's chat message
    - **target_language**: "en" or "ur"
    - **selected_text**: Optional text selection for context
    - **conversation_history**: Previous messages for context
    - **user_id**: Optional user ID for logging
    """
    try:
        # Check if OpenAI is configured
        if not openai_client:
            raise HTTPException(
                status_code=500,
                detail="OpenAI API not configured. Please set OPENAI_API_KEY in environment variables."
            )

        # Determine query - use selected text if provided
        query = request.selected_text if request.selected_text else request.message

        # Step 1: Retrieve relevant context from Qdrant
        print(f"🔍 Searching Qdrant for: {query[:50]}...")

        search_results = search_module.search(
            query=query,
            limit=5,
            score_threshold=0.5  # Lower threshold for better recall
        )

        print(f"✅ Found {len(search_results)} relevant documents")

        # Build context from search results
        context_parts = []
        sources = []

        for result in search_results:
            context_parts.append(f"[Source: {result['url']}]\n{result['content']}")
            sources.append({
                "url": result["url"],
                "score": result["score"],
                "content_preview": result["content"][:200] + "..."
            })

        context = "\n\n---\n\n".join(context_parts) if context_parts else "No specific context found."

        # Step 2: Build system message for OpenAI
        if request.target_language == 'ur':
            system_instruction = f"""You are a helpful assistant for a Humanoid Robotics and Physical AI learning platform.

Your task is to answer questions STRICTLY based on the provided context from the book, then translate your answer into clear, simple Urdu.

Context from the book:
{context}

CRITICAL INSTRUCTIONS - FOLLOW EXACTLY:
1. First, understand the question and find the answer using ONLY the provided context above
2. If the context contains the answer, formulate a clear response based on that context
3. If the context does NOT contain the answer, politely say you don't have that information in the provided material
4. Then, translate your entire response into clear, simple Urdu
5. Your FINAL OUTPUT must be ONLY in Urdu - absolutely NO English words or phrases
6. Do NOT add information not present in the context
7. Do NOT hallucinate or make up facts
8. Be concise but informative (2-4 sentences in Urdu)
9. Use appropriate Urdu technical terms when available, otherwise transliterate English technical terms into Urdu script

If context is insufficient, respond in Urdu: "معاف کیجیے، فراہم کردہ مواد میں اس سوال کا جواب موجود نہیں ہے۔"
"""
        else:
            system_instruction = f"""You are a helpful assistant for a Humanoid Robotics and Physical AI learning platform.

Answer questions based STRICTLY on the provided context from the book.

Context from the book:
{context}

INSTRUCTIONS:
- Answer in English using ONLY the information provided in the context above
- If the context doesn't contain the answer, politely state that the information is not available in the provided material
- Do NOT add information not present in the context
- Do NOT hallucinate or make up facts
- Be concise but informative (2-4 sentences)
- Use technical terms appropriately
- Be encouraging and educational

If context is insufficient, respond: "I don't have that information in the provided book content."
"""

        if request.selected_text:
            system_instruction += f"\n\nThe user selected this specific text:\n{request.selected_text}\n\nAnswer their question about this specific text."

        # Build conversation messages for OpenAI
        messages = [{"role": "system", "content": system_instruction}]

        # Add conversation history
        if request.conversation_history:
            for msg in request.conversation_history[-6:]:  # Last 3 exchanges
                role = msg.get("role", "user")
                content = msg.get("content", "")
                if role in ["user", "assistant"]:
                    messages.append({"role": role, "content": content})

        # Add current message
        messages.append({"role": "user", "content": request.message})

        # Step 3: Get response from OpenAI
        print("🤖 Generating response with OpenAI...")

        response = openai_client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=messages,
            temperature=0.7,
            max_tokens=500,
        )

        response_text = response.choices[0].message.content

        print(f"✅ Response generated: {response_text[:50]}...")

        # Log chat interaction to database if user is provided
        if request.user_id:
            try:
                user = get_user_by_id(db, request.user_id)
                if user:
                    chat_history = ChatHistory(
                        user_id=request.user_id,
                        message=request.message,
                        response=response_text,
                        language=request.target_language,
                        sources=[{"url": s["url"], "score": s["score"]} for s in sources]
                    )
                    db.add(chat_history)
                    db.commit()
                    print(f"💾 Chat logged for user {request.user_id}")
            except Exception as e:
                print(f"⚠️  Failed to log chat: {e}")

        return ChatResponse(
            response=response_text,
            sources=sources,
            language=request.target_language
        )

    except Exception as e:
        print(f"❌ Chat error: {e}")
        import traceback
        traceback.print_exc()

        # Return user-friendly error
        error_message = (
            "معذرت، میں اس وقت جواب نہیں دے سکتا۔ براہ کرم بعد میں دوبارہ کوشش کریں۔"
            if request.target_language == "ur"
            else "Sorry, I couldn't process your request at this time. Please try again."
        )

        raise HTTPException(
            status_code=500,
            detail={"message": error_message, "error": str(e)}
        )

@router.post("/chat/personalize", response_model=PersonalizeResponse)
async def personalize_content(
    request: PersonalizeRequest,
    db: Session = Depends(get_db)
):
    """
    Personalize content based on user background

    **Features:**
    - Adapts content complexity based on user experience
    - Adjusts explanations based on user background
    - Saves personalization preferences to database

    **Parameters:**
    - **content**: Original content to personalize
    - **user_background**: User's background information (software_experience, hardware_experience, etc.)
    - **chapter_id**: Optional chapter ID for tracking
    - **user_id**: Optional user ID for saving preferences
    """
    try:
        # Check if OpenAI is configured
        if not openai_client:
            raise HTTPException(
                status_code=500,
                detail="OpenAI API not configured. Please set OPENAI_API_KEY in environment variables."
            )

        # Build system message for personalization
        system_instruction = f"""You are an AI content personalization assistant for a Humanoid Robotics and Physical AI learning platform.

Your task is to adapt the provided content based on the user's background to make it more relevant and understandable.

User's Background:
- Software Experience: {request.user_background.get('software_experience', 'not specified')}
- Hardware Experience: {request.user_background.get('hardware_experience', 'not specified')}
- Programming Level: {request.user_background.get('programming_level', 'not specified')}
- Learning Goals: {request.user_background.get('learning_goals', 'not specified')}

Content to Personalize:
{request.content}

PERSONALIZATION RULES:
1. If user is a beginner: Add more context, simpler explanations, analogies
2. If user is advanced/expert: Add more technical depth, advanced concepts
3. If user is hardware-focused: Emphasize practical implementation, circuits, sensors
4. If user is software-focused: Emphasize code examples, algorithms, APIs
5. Maintain the core educational value and accuracy of the content
6. Adjust complexity, examples, and focus areas based on user background
7. Do NOT change the factual information, only the presentation and depth
8. Keep the personalized content the same length or similar to original
"""

        # Build conversation messages for OpenAI
        messages = [
            {"role": "system", "content": system_instruction},
            {"role": "user", "content": f"Please personalize the content based on the user's background."}
        ]

        # Get response from OpenAI
        print("🤖 Generating personalized content with OpenAI...")

        response = openai_client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=messages,
            temperature=0.7,
            max_tokens=2000,  # Higher token limit for longer content
        )

        personalized_content = response.choices[0].message.content

        print(f"✅ Personalized content generated")

        # Save personalization preference to database if user_id and chapter_id provided
        if request.user_id and request.chapter_id:
            try:
                save_content_preference(
                    db,
                    user_id=request.user_id,
                    chapter_id=request.chapter_id,
                    personalized=True,
                    translated=False,
                    target_language="en"
                )
                print(f"💾 Personalization preference saved for user {request.user_id}, chapter {request.chapter_id}")
            except Exception as e:
                print(f"⚠️  Failed to save personalization preference: {e}")

        return PersonalizeResponse(
            success=True,
            personalized_content=personalized_content
        )

    except Exception as e:
        print(f"❌ Personalization error: {e}")
        import traceback
        traceback.print_exc()

        raise HTTPException(
            status_code=500,
            detail={"success": False, "error": str(e)}
        )

@router.post("/chat/translate", response_model=TranslateResponse)
async def translate_content(
    request: TranslateRequest
):
    """
    Translate content to target language using OpenAI

    **Parameters:**
    - **content**: Content to translate
    - **target_language**: Target language code (e.g., 'ur' for Urdu)
    """
    try:
        # Check if OpenAI is configured
        if not openai_client:
            raise HTTPException(
                status_code=500,
                detail="OpenAI API not configured. Please set OPENAI_API_KEY in environment variables."
            )

        # Determine target language instruction
        if request.target_language == 'ur':
            system_instruction = f"""You are a professional translator for a Humanoid Robotics and Physical AI learning platform.

Your task is to translate the provided educational content into clear, accurate Urdu while preserving technical terminology.

Content to Translate:
{request.content}

TRANSLATION RULES:
1. Translate the content into clear, natural Urdu
2. Preserve technical terms - either keep them in English or use appropriate Urdu equivalents
3. Maintain the educational tone and meaning
4. Ensure the translation is readable and understandable
5. Do NOT add explanations not present in the original text
6. If technical terms don't have good Urdu equivalents, keep them in English script
7. Follow proper Urdu grammar and sentence structure
8. Preserve the structure and formatting as much as possible
"""
        else:
            # Default to English or other language translation
            system_instruction = f"""You are a professional translator for a Humanoid Robotics and Physical AI learning platform.

Your task is to translate the provided educational content into {request.target_language} while preserving technical terminology.

Content to Translate:
{request.content}

TRANSLATION RULES:
1. Translate the content into clear, natural {request.target_language}
2. Preserve technical terms - either keep them in English or use appropriate {request.target_language} equivalents
3. Maintain the educational tone and meaning
4. Ensure the translation is readable and understandable
5. Do NOT add explanations not present in the original text
6. If technical terms don't have good {request.target_language} equivalents, keep them in English
7. Preserve the structure and formatting as much as possible
"""

        # Build conversation messages for OpenAI
        messages = [
            {"role": "system", "content": system_instruction},
            {"role": "user", "content": f"Please translate the content to the target language."}
        ]

        # Get response from OpenAI
        print(f"🌐 Translating content to {request.target_language} with OpenAI...")

        response = openai_client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=messages,
            temperature=0.3,  # Lower temperature for more consistent translation
            max_tokens=2000,
        )

        translated_text = response.choices[0].message.content

        print(f"✅ Content translated to {request.target_language}")

        return TranslateResponse(
            success=True,
            translated_text=translated_text
        )

    except Exception as e:
        print(f"❌ Translation error: {e}")
        import traceback
        traceback.print_exc()

        raise HTTPException(
            status_code=500,
            detail={"success": False, "error": str(e)}
        )

# Export router
__all__ = ["router"]
