import React from 'react';
import { useNavigate } from '@docusaurus/router';

export default function LessonCard({ lesson, progress, chapterId }) {
  const navigate = useNavigate();

  return (
    <div
      className="bg-white border border-gray-200 rounded-lg p-5 hover:shadow-md transition-all duration-200 cursor-pointer"
      onClick={() => navigate(`/docs/chapters/${chapterId}/${lesson.id}`)}
    >
      <div className="flex justify-between items-start">
        <div className="flex-1">
          <h3 className="font-semibold text-gray-900 mb-2">{lesson.title}</h3>
          {lesson.description && (
            <p className="text-sm text-gray-600 mb-3">{lesson.description}</p>
          )}
          <div className="flex items-center text-xs text-gray-500">
            <span className="bg-gray-100 px-2 py-1 rounded mr-2">Lesson</span>
            {lesson.duration && <span>{lesson.duration} min read</span>}
          </div>
        </div>
        {progress !== undefined && (
          <div className="ml-4 w-12">
            <div className="w-full bg-gray-200 rounded-full h-2">
              <div
                className="bg-blue-600 h-2 rounded-full"
                style={{ width: `${progress}%` }}
              ></div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}