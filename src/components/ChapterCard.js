import React from 'react';
import { useNavigate } from '@docusaurus/router';

export default function ChapterCard({ chapter, progress }) {
  const navigate = useNavigate();

  return (
    <div
      className="bg-white border border-gray-200 rounded-lg p-6 hover:shadow-lg transition-all duration-200 cursor-pointer transform hover:-translate-y-1"
      onClick={() => navigate(`/docs/chapters/${chapter.id}`)}
    >
      <div className="flex items-start justify-between mb-4">
        <div>
          <h3 className="text-xl font-bold text-gray-900 mb-2">{chapter.title}</h3>
          <p className="text-gray-600 text-sm">{chapter.description}</p>
        </div>
        <div className="bg-blue-100 text-blue-800 px-3 py-1 rounded-full text-sm font-medium">
          {chapter.lessons.length} lessons
        </div>
      </div>

      {progress && (
        <div className="mt-4">
          <div className="flex justify-between text-sm text-gray-600 mb-1">
            <span>Progress</span>
            <span>{progress}%</span>
          </div>
          <div className="w-full bg-gray-200 rounded-full h-2">
            <div
              className="bg-blue-600 h-2 rounded-full transition-all duration-500 ease-out"
              style={{ width: `${progress}%` }}
            ></div>
          </div>
        </div>
      )}
    </div>
  );
}