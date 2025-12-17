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
from database import get_db, ChatHistory, get_user_by_id

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

# Export router
__all__ = ["router"]
