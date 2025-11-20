#!/usr/bin/env python3
"""
Script to parse quiz document and extract question-answer pairs to CSV format.
"""

import re
import csv
from bs4 import BeautifulSoup


def clean_text(text):
    """Remove extra whitespace and clean up text."""
    # Replace multiple spaces/newlines with single space
    text = re.sub(r'\s+', ' ', text)
    return text.strip()


def parse_quiz_document(html_content):
    """
    Parse the HTML quiz document and extract question-answer pairs.
    
    Returns:
        List of tuples: [(question, answer), ...]
    """
    soup = BeautifulSoup(html_content, 'html.parser')
    qa_pairs = []
    
    # Get all paragraph elements
    paragraphs = soup.find_all('p', class_='MsoNormal')
    
    current_question = ""
    
    for i, p in enumerate(paragraphs):
        text = p.get_text()
        
        # Check if this paragraph contains an answer
        if 'Answer:' in text:
            # Extract the answer (underlined text)
            underlined = p.find_all('u')
            if underlined:
                # Get all underlined answers (some questions have multiple parts)
                answers = [clean_text(u.get_text()) for u in underlined]
                answer = ', '.join(answers)
                
                # Clean up the current question
                if current_question:
                    question = clean_text(current_question)
                    qa_pairs.append((question, answer))
                    current_question = ""
        else:
            # This is part of a question
            cleaned = clean_text(text)
            # Skip empty lines or lines that are just non-breaking spaces
            if cleaned and cleaned != '':
                # Check if this is a new numbered question (1), 2), etc.)
                if re.match(r'^\d+\)', cleaned):
                    # If we have a previous question without answer, save it
                    if current_question:
                        current_question = ""
                    current_question = cleaned
                else:
                    # This is a continuation or sub-question
                    if current_question:
                        current_question += " " + cleaned
                    else:
                        current_question = cleaned
    
    return qa_pairs


def write_to_csv(qa_pairs, output_file='quiz_qa.csv'):
    """
    Write question-answer pairs to CSV file with semicolon delimiter.
    
    Args:
        qa_pairs: List of (question, answer) tuples
        output_file: Output CSV filename
    """
    with open(output_file, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f, delimiter=';')
        writer.writerow(['Question', 'Answer'])  # Header
        writer.writerows(qa_pairs)
    
    print(f"Successfully wrote {len(qa_pairs)} Q&A pairs to {output_file}")


def main():
    # Read the input file
    input_file = 'doc.txt'  # Update this path
    
    try:
        with open(input_file, 'r', encoding='utf-8') as f:
            html_content = f.read()
        
        # Parse the document
        qa_pairs = parse_quiz_document(html_content)
        
        # Write to CSV
        write_to_csv(qa_pairs, 'quiz_qa.csv')
        
        # Preview first few pairs
        print("\nFirst 5 Q&A pairs:")
        for i, (q, a) in enumerate(qa_pairs[:5], 1):
            print(f"\n{i}. Q: {q[:100]}...")
            print(f"   A: {a}")
        
    except FileNotFoundError:
        print(f"Error: Could not find file {input_file}")
        print("Please update the input_file path in the script.")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()