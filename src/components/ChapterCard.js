import React from 'react';
import { useNavigate } from '@docusaurus/router';
import styled from 'styled-components';

const CardContainer = styled.div`
  background-color: #fff;
  border: 1px solid #e5e7eb;
  border-radius: 0.5rem;
  padding: 1.5rem;
  cursor: pointer;
  transition: all 0.2s ease;
  box-shadow: 0 1px 3px 0 rgba(0, 0, 0, 0.1), 0 1px 2px -1px rgba(0, 0, 0, 0.1);

  &:hover {
    box-shadow: 0 10px 15px -3px rgba(0, 0, 0, 0.1), 0 4px 6px -4px rgba(0, 0, 0, 0.1);
    transform: translateY(-0.25rem);
  }
`;

const CardHeader = styled.div`
  display: flex;
  align-items: flex-start;
  justify-content: space-between;
  margin-bottom: 1rem;
`;

const CardContent = styled.div`
  flex: 1;
`;

const CardTitle = styled.h3`
  font-size: 1.25rem;
  font-weight: 700;
  color: #1f2937;
  margin-bottom: 0.5rem;
`;

const CardDescription = styled.p`
  font-size: 0.875rem;
  color: #4b5563;
`;

const LessonBadge = styled.div`
  background-color: #dbeafe;
  color: #1e40af;
  padding: 0.25rem 0.75rem;
  border-radius: 9999px;
  font-size: 0.875rem;
  font-weight: 500;
`;

const ProgressContainer = styled.div`
  margin-top: 1rem;
`;

const ProgressHeader = styled.div`
  display: flex;
  justify-content: space-between;
  font-size: 0.875rem;
  color: #4b5563;
  margin-bottom: 0.25rem;
`;

const ProgressBar = styled.div`
  width: 100%;
  background-color: #e5e7eb;
  border-radius: 9999px;
  height: 0.5rem;
`;

const ProgressFill = styled.div`
  background-color: #2563eb;
  height: 100%;
  border-radius: 9999px;
  transition: width 0.5s ease-out;
  width: ${props => props.progress}%;
`;

export default function ChapterCard({ chapter, progress }) {
  const navigate = useNavigate();

  return (
    <CardContainer onClick={() => navigate(`/docs/chapters/${chapter.id}`)}>
      <CardHeader>
        <CardContent>
          <CardTitle>{chapter.title}</CardTitle>
          <CardDescription>{chapter.description}</CardDescription>
        </CardContent>
        <LessonBadge>
          {chapter.lessons.length} lessons
        </LessonBadge>
      </CardHeader>

      {progress && (
        <ProgressContainer>
          <ProgressHeader>
            <span>Progress</span>
            <span>{progress}%</span>
          </ProgressHeader>
          <ProgressBar>
            <ProgressFill progress={progress} />
          </ProgressBar>
        </ProgressContainer>
      )}
    </CardContainer>
  );
}